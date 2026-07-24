//
// OpenVR Capture
// Forked by pigney
// Originally "OpenVR Capture input plugin for OBS" by Keijo "Kegetys" Ruotsalainen
//

#include <obs-module.h>
#include <graphics/image-file.h>
#include <util/platform.h>
#include <util/dstr.h>
#include <sys/stat.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <stdint.h>
#include <algorithm>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <wrl/client.h>
#include "headers/openvr.h"
using Microsoft::WRL::ComPtr;

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dcompiler.lib")
#pragma comment(lib, "lib/win64/openvr_api.lib")

static bool init_inprog = false;
static bool IsVRSystemInitialized = false;

std::chrono::steady_clock::time_point last_init_time = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point last_init_timeBUFFER = std::chrono::steady_clock::now();
static constexpr std::chrono::milliseconds retry_delay{8}; // update at ~120Hz
static constexpr std::chrono::milliseconds retry_delayBUFFER{500}; // init at 2Hz

#define blog(log_level, message, ...) \
	blog(log_level, "[win_openvr] " message, ##__VA_ARGS__)
#define debug(message, ...)                                                    \
	blog(LOG_DEBUG, "[%s] " message, obs_source_get_name(context->source), \
	     ##__VA_ARGS__)
#define info(message, ...)                                                    \
	blog(LOG_INFO, "[%s] " message, obs_source_get_name(context->source), \
	     ##__VA_ARGS__)
#define warn(message, ...)                 \
	blog(LOG_WARNING, "[%s] " message, \
	     obs_source_get_name(context->source), ##__VA_ARGS__)

// --- Quaternion / vector helpers for stabilization ---

struct quatd {
	double w, x, y, z;
};
struct vec3d {
	double x, y, z;
};

static quatd quat_from_mat3(const double m[3][3])
{
	// Rotation matrix (column-vector convention) -> quaternion, Shepperd's method
	double tr = m[0][0] + m[1][1] + m[2][2];
	quatd q;
	if (tr > 0.0) {
		double s = sqrt(tr + 1.0) * 2.0;
		q.w = 0.25 * s;
		q.x = (m[2][1] - m[1][2]) / s;
		q.y = (m[0][2] - m[2][0]) / s;
		q.z = (m[1][0] - m[0][1]) / s;
	} else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
		double s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
		q.w = (m[2][1] - m[1][2]) / s;
		q.x = 0.25 * s;
		q.y = (m[0][1] + m[1][0]) / s;
		q.z = (m[0][2] + m[2][0]) / s;
	} else if (m[1][1] > m[2][2]) {
		double s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
		q.w = (m[0][2] - m[2][0]) / s;
		q.x = (m[0][1] + m[1][0]) / s;
		q.y = 0.25 * s;
		q.z = (m[1][2] + m[2][1]) / s;
	} else {
		double s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
		q.w = (m[1][0] - m[0][1]) / s;
		q.x = (m[0][2] + m[2][0]) / s;
		q.y = (m[1][2] + m[2][1]) / s;
		q.z = 0.25 * s;
	}
	return q;
}

static quatd quat_from_hmd34(const vr::HmdMatrix34_t &mm)
{
	// Upper 3x3 of the row-major device-to-absolute transform
	const double m[3][3] = {{mm.m[0][0], mm.m[0][1], mm.m[0][2]},
				{mm.m[1][0], mm.m[1][1], mm.m[1][2]},
				{mm.m[2][0], mm.m[2][1], mm.m[2][2]}};
	return quat_from_mat3(m);
}

static quatd quat_conj(const quatd &q)
{
	return {q.w, -q.x, -q.y, -q.z};
}

static double quat_dot(const quatd &a, const quatd &b)
{
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

static quatd quat_normalize(const quatd &q)
{
	double n = sqrt(quat_dot(q, q));
	if (!(n > 1e-12)) // negated compare also catches NaN
		return {1.0, 0.0, 0.0, 0.0};
	return {q.w / n, q.x / n, q.y / n, q.z / n};
}

static quatd quat_mul(const quatd &a, const quatd &b)
{
	return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

static quatd quat_slerp(const quatd &a, quatd b, double t)
{
	double d = quat_dot(a, b);
	if (d < 0.0) {
		b = {-b.w, -b.x, -b.y, -b.z};
		d = -d;
	}
	if (d > 0.9995) {
		quatd r = {a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
			   a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
		return quat_normalize(r);
	}
	double th = acos(d);
	double s = sin(th);
	double wa = sin((1.0 - t) * th) / s;
	double wb = sin(t * th) / s;
	return {wa * a.w + wb * b.w, wa * a.x + wb * b.x,
		wa * a.y + wb * b.y, wa * a.z + wb * b.z};
}

static vec3d quat_rotate(const quatd &q, const vec3d &v)
{
	// v' = v + w*t + q_vec x t, where t = 2 * q_vec x v
	vec3d t = {2.0 * (q.y * v.z - q.z * v.y), 2.0 * (q.z * v.x - q.x * v.z),
		   2.0 * (q.x * v.y - q.y * v.x)};
	return {v.x + q.w * t.x + (q.y * t.z - q.z * t.y),
		v.y + q.w * t.y + (q.z * t.x - q.x * t.z),
		v.z + q.w * t.z + (q.x * t.y - q.y * t.x)};
}

// Remove the twist about the view axis so the camera's horizon is level
// (world +y up), keeping the forward direction unchanged. Near-vertical
// views fade back to the unleveled pose to avoid a gimbal snap.
static quatd quat_level_roll(const quatd &q)
{
	const vec3d f = quat_rotate(q, vec3d{0.0, 0.0, -1.0});
	const double fy = fabs(f.y);
	if (!(fy < 0.98)) // near-vertical or NaN
		return q;

	// Level basis: back = -f, right = up x back, up' = back x right
	const vec3d b = {-f.x, -f.y, -f.z};
	vec3d x = {b.z, 0.0, -b.x}; // (0,1,0) x b
	const double xn = sqrt(x.x * x.x + x.z * x.z);
	if (!(xn > 1e-9))
		return q;
	x = {x.x / xn, 0.0, x.z / xn};
	const vec3d y = {b.y * x.z - b.z * x.y, b.z * x.x - b.x * x.z, b.x * x.y - b.y * x.x};
	const double m[3][3] = {{x.x, y.x, b.x}, {x.y, y.y, b.y}, {x.z, y.z, b.z}};
	quatd ql = quat_normalize(quat_from_mat3(m));

	// Fade the lock out between 0.85 and 0.98 of vertical
	if (fy > 0.85)
		ql = quat_slerp(ql, q, (fy - 0.85) / (0.98 - 0.85));
	return quat_normalize(ql);
}

// Project an eye-frustum corner ray through the eye-to-head rotation onto
// the head-space tangent plane. False for degenerate directions.
static bool eye_corner_head_tangent(const quatd &q_e2h, double tx, double ty, double *px, double *py)
{
	const vec3d d = quat_rotate(q_e2h, vec3d{tx, ty, -1.0});
	if (!(d.z < -0.1))
		return false;
	*px = d.x / -d.z;
	*py = d.y / -d.z;
	return true;
}

// Debug aid: pin the camera to the horizon - zero pitch AND roll, yaw still
// follows the head. Gives pitch an ABSOLUTE (gravity) target the way Roll
// Lock does for roll: every filter target has exactly zero pitch, so under
// head nodding any residual vertical motion of the horizon line isolates
// warp-gain / pose-pairing error from filter lag.
static quatd quat_level_horizon(const quatd &q)
{
	const vec3d f = quat_rotate(q, vec3d{0.0, 0.0, -1.0});
	const double n = sqrt(f.x * f.x + f.z * f.z);
	if (!(n > 1e-6)) // looking straight up/down (or NaN): fall back
		return quat_level_roll(q);
	// Level basis: back = -f_horizontal, up = world +y, right = up x back
	const vec3d b = {-f.x / n, 0.0, -f.z / n};
	const vec3d x = {b.z, 0.0, -b.x};
	const double m[3][3] = {{x.x, 0.0, b.x}, {0.0, 1.0, 0.0}, {x.z, 0.0, b.z}};
	return quat_normalize(quat_from_mat3(m));
}

// Row-major 4x4 (upper 3x3 = rotation, column-vector convention)
static void quat_to_mat4(const quatd &q, float *m)
{
	const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
	const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
	const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
	const double r[9] = {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
			     2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
			     2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)};
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			m[i * 4 + j] = (i < 3 && j < 3) ? (float)r[i * 3 + j] : ((i == j) ? 1.0f : 0.0f);
}

// Reprojection / composite pass: output pixel -> ray of the output camera ->
// rotate into the actual camera frame -> reproject with each eye's asymmetric
// frustum -> sample the mirror texture(s). Exact for rotation incl. roll.
// Single-eye mode: rays live in the (dominant) eye's space, frC == frA.
// Wide mode: rays live in HEAD space on a union-frustum canvas; both eyes are
// sampled at infinity alignment (IPD ignored) and blended with the dominant
// eye (A) winning everywhere it has coverage - eye B fades in over a narrow
// band at A's nasal edge, so the seam sits far off-center.
// Compiled with D3DCOMPILE_PACK_MATRIX_ROW_MAJOR so mul(M, v) is R*v for the
// row-major matrix uploaded from quat_to_mat4.
static const char *STAB_SHADER_SRC = R"hlsl(
cbuffer StabParams : register(b0)
{
	float4 frC;   // output canvas frustum tangents: l, r, t, b
	float4 frA;   // eye A (dominant / single) frustum
	float4 frB;   // eye B frustum (wide mode only)
	float4 crop;  // crop rect in canvas uv: u0, v0, du, dv
	float4 misc;  // x: re-encode srgb, y: wide mode, z: blend band (tangent units),
		      // w: +1 A's nasal edge is its left, -1 it's its right
	float4 poscA; // xyz: positional correction / reference depth (eye A space)
	float4 poscB; // xyz: same, eye B space
	float4x4 RdA; // eye A <- smoothed rotation
	float4x4 RdB; // eye B <- smoothed rotation
};
Texture2D imgA : register(t0);
Texture2D imgB : register(t1);
SamplerState smp : register(s0);

struct VSOut
{
	float4 pos : SV_Position;
	float2 uv : TEXCOORD0;
};

VSOut VSMain(uint id : SV_VertexID)
{
	VSOut o;
	float2 uv = float2((id << 1) & 2, id & 2);
	o.uv = uv;
	o.pos = float4(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.0, 1.0);
	return o;
}

float3 lin_to_srgb(float3 c)
{
	c = saturate(c);
	return (c <= 0.0031308) ? c * 12.92 : 1.055 * pow(c, 1.0 / 2.4) - 0.055;
}

float4 PSMain(VSOut i) : SV_Target
{
	float2 uvV = crop.xy + i.uv * crop.zw;
	float tx = frC.x + uvV.x * (frC.y - frC.x);
	float ty = frC.w - uvV.y * (frC.w - frC.z);
	float3 ray = float3(tx, ty, -1.0);

	float3 dA = mul((float3x3)RdA, ray) - poscA.xyz;
	float2 tA = dA.xy / max(-dA.z, 1e-6);
	float2 uvA = float2((tA.x - frA.x) / (frA.y - frA.x), (frA.w - tA.y) / (frA.w - frA.z));

	float4 c;
	if (misc.y < 0.5) {
		c = imgA.Sample(smp, uvA);
	} else {
		float3 dB = mul((float3x3)RdB, ray) - poscB.xyz;
		float2 tB = dB.xy / max(-dB.z, 1e-6);
		float2 uvB = float2((tB.x - frB.x) / (frB.y - frB.x), (frB.w - tB.y) / (frB.w - frB.z));

		float inA = (dA.z < 0.0 && uvA.x >= 0.0 && uvA.x <= 1.0 && uvA.y >= 0.0 && uvA.y <= 1.0) ? 1.0 : 0.0;
		float inB = (dB.z < 0.0 && uvB.x >= 0.0 && uvB.x <= 1.0 && uvB.y >= 0.0 && uvB.y <= 1.0) ? 1.0 : 0.0;
		// Distance (in tangent units) from each eye's NASAL edge; the
		// dominant eye holds full weight until the ray nears the edge of
		// its own coverage, then hands off to B across the band.
		float nasA = (misc.w > 0.0) ? (tA.x - frA.x) : (frA.y - tA.x);
		float nasB = (misc.w > 0.0) ? (frB.y - tB.x) : (tB.x - frB.x);
		float wA = inA * saturate(nasA / misc.z);
		float wB = inB * saturate(nasB / misc.z) * (1.0 - wA);
		float s = wA + wB;
		c = (s > 1e-4) ? (imgA.Sample(smp, uvA) * wA + imgB.Sample(smp, uvB) * wB) / s
			       : float4(0.0, 0.0, 0.0, 1.0);
	}
	if (misc.x > 0.5)
		c.rgb = lin_to_srgb(c.rgb);
	return c;
}
)hlsl";

// Width of the wide-mode blend band at the dominant eye's nasal edge, in
// tangent units (~4.5 degrees).
#define STAB_BLEND_BAND 0.08f

// Auto pose-delay vote window, in compositor frames (~1.3 s at 90 fps).
#define STAB_AUTO_WIN 120

struct StabCBData {
	float frC[4];
	float frA[4];
	float frB[4];
	float crop[4];
	float misc[4];
	float poscA[4];
	float poscB[4];
	float RdA[16];
	float RdB[16];
};

struct win_openvr {
	obs_source_t *source;

	int eye_mode;  // 0 = left, 1 = right, 2 = wide (left dom), 3 = wide (right dom)
	bool righteye; // dominant eye is the right one
	bool wide_active;
	double active_aspect_ratio;
	bool ar_crop;

	uint32_t lastFrame;

	gs_texture_t *texture;
	//ComPtr<ID3D11Device> dev11;
	//ComPtr<ID3D11DeviceContext> ctx11;
	ComPtr<ID3D11Resource> tex;
	ComPtr<ID3D11ShaderResourceView> mirrorSrv;
	ComPtr<ID3D11ShaderResourceView> mirrorSrv2; // non-dominant eye (wide mode)
	ComPtr<ID3D11Device> shared_device = nullptr;
	ComPtr<ID3D11DeviceContext> shared_context = nullptr;

	ComPtr<IDXGIResource> res;

	ComPtr<ID3D11Texture2D> texCrop;

	// Set in win_openvr_init, 0 until then.
	unsigned int device_width;
	unsigned int device_height;

	unsigned int x;
	unsigned int y;
	unsigned int width;
	unsigned int height;

	double scale_factor;
	int x_offset;
	int y_offset;

	bool initialized;
	bool active;

	// Stabilization settings
	bool stabilize;
	double stab_tau; // smoothing time constant, seconds
	int stab_filter; // 0 = damped average, 1 = One Euro (adaptive)
	bool stab_roll_lock;
	bool stab_horizon_lock; // debug: absolute pitch+roll anchor
	bool stab_full_lock;    // debug: freeze all axes to a captured heading
	quatd stab_lock_ref;    // captured full-lock target
	bool stab_lock_captured;
	bool stab_pos_comp;
	double stab_pos_depth;
	int stab_pose_delay;
	bool stab_debug;
	bool stab_telemetry;
	FILE *stab_telemetry_file;
	uint32_t stab_telemetry_lines;

	quatd stab_q_ref; // raw pose of the displayed frame (telemetry)

	// Pairing: how many compositor frames the mirror lags the in-flight frame
	// (the pose each mirror frame is paired with). The live mirror is warped
	// against GetFrameTiming(stab_pair_frames)'s pose - no buffering needed.
	int stab_pair_frames;

	// Boundary-count pairing. The mirror lag is a fixed DURATION (stab_pair_lag_ms),
	// not a fixed number of timing records: the compositor runs on a hard ~120 Hz
	// vsync grid and a record spans an INTEGER number of vsyncs - 1 at idle, 2 under
	// game load. So the same physical lag is a different record count at each load,
	// which is why no fixed integer offset works at both.
	bool stab_pair_auto;
	double stab_pair_lag_ms;
	uint32_t stab_last_sel; // frame index of the record paired with last frame
	bool stab_has_sel;
	double stab_dframe_avg;  // slow average of records-per-sample (the phase proxy)
	bool stab_pair_phase;    // use the measured sub-record sampling phase
	double stab_rec_period;  // measured seconds per compositor record
	double stab_last_t0;     // previous sample's newest-record timestamp

	// Auto Mirror Lag. Native (app at refresh): calibrated +8.5 ms. Under async
	// reprojection the mirror sits one record behind its timestamp: calibrated
	// -8.5, exactly one 2-vsync record lower. lag = base - (n >= 2 ? n*V : 0),
	// where n = integer vsyncs per record and V = vsync period - QUANTIZED to
	// the vsync grid, never the raw period average, which is contaminated by
	// genuine 3-vsync gaps (8-24% in game) and would wander the lag near a
	// record boundary where selection alternates (= shake).
	bool stab_lag_auto;
	double stab_lag_base_ms; // native lag; own key so a stale manual slider cannot poison auto
	int stab_lag_n;          // classifier: vsyncs per record (1 = native)
	int stab_lag_dwell;      // ticks the candidate cadence has persisted
	double stab_vsync_s;     // cached 1/Prop_DisplayFrequency_Float, 0 = unknown
	int stab_vsync_wait;     // probe backoff after a failed query
	bool stab_vsync_warned;

	// Boundary poller. We render on OBS's tick, which is SLOWER than the
	// compositor produces frames (2 records per look at 120/s), so from the
	// graphics thread alone we cannot see WHEN a frame boundary happened - only
	// that one did. This thread watches m_nFrameIndex at ~1 kHz and stamps QPC
	// at each change, which makes the sub-frame sampling phase a measured
	// quantity instead of one inferred from an API whose semantics we had to
	// guess. It touches no D3D and never the mirror SRV.
	// Raw Win32 primitives, not std::thread/mutex: the context is bzalloc'd and
	// never constructed, so C++ objects with non-trivial ctors would be UB here.
	// The ring is lock-free by construction - aligned 32/64-bit scalars are
	// atomic on x64, one writer, one reader, and a stale entry is harmless.
	HANDLE stab_poll_thread;
	volatile LONG stab_poll_run;
	volatile LONG stab_poll_pos;
	uint32_t stab_poll_idx[16];
	int64_t stab_poll_qpc[16];
	int64_t stab_qpc_freq;

	// Boundary-locked capture: the poller copies the mirror at MID-SLOT
	// (boundary + V/2) into a 2-slot ring, fixing the capture phase at ANY
	// canvas rate - reproducing what made canvas-120 steady (capture landing
	// just after each record) without depending on the canvas at all.
	// Mid-slot and not at the boundary: the native mirror update lands ~0.5 ms
	// after the boundary, and capturing there would park ON the race forever.
	SRWLOCK stab_gpu_lock;     // guards ALL shared_context use; SRWLOCK_INIT is {0}
	volatile LONG stab_cap_on; // poller capture enabled (hold on, not degraded)
	int64_t stab_cap_due;      // scheduled mid-slot copy instant (QPC); 0 = none
	ID3D11Texture2D *cap_tex[2][2]; // [slot][eye] - raw pointers: bzalloc-safe
	ID3D11ShaderResourceView *cap_srv[2][2];
	int64_t cap_qpc[2];  // capture stamp (copy-issue instant)
	uint32_t cap_idx[2]; // newest record index at capture
	double cap_phase_ms[2];
	volatile LONG cap_valid[2];
	int cap_newest;
	// Poller health: Sleep(1) silently degrades to ~15.6 ms under Windows
	// timer coalescing, and systematically-late stamps are WORSE than tick
	// capture - so self-measure and fall back rather than mispair quietly.
	int64_t poll_prev_wake;
	int poll_slow, poll_seen;
	bool poll_degraded;
	int stab_vsync_age; // re-probe the refresh every ~10 s (it can change)
	uint32_t stab_last_dframe;
	double stab_extrap_s;    // seconds to extrapolate the paired pose FORWARD

	// One-frame hold. The GPU samples the mirror some ms after we read the timing
	// on the CPU, so the record describing what we captured may not exist yet -
	// which forces extrapolation, and extrapolation overshoots at direction
	// changes (measured ~3 deg at p99). Holding the captured pixels for one tick
	// means that record HAS arrived, so the pose is looked up, not predicted.
	bool stab_hold;
	Microsoft::WRL::ComPtr<ID3D11Texture2D> texHold, texHold2;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> holdSrv, holdSrv2;
	bool hold_valid;
	int64_t hold_qpc;    // QPC at the instant the held pixels were captured
	uint32_t hold_frame; // newest record index at that instant
	int stab_pair_auto_cur;  // offset currently applied
	int stab_pair_auto_cand; // candidate awaiting hysteresis confirmation
	int stab_pair_auto_hits;

	// Per-eye projection frustum tangents from GetProjectionRaw
	float proj_left, proj_right, proj_top, proj_bottom;     // dominant eye
	float proj2_left, proj2_right, proj2_top, proj2_bottom; // other eye (wide)
	// Output canvas frustum + pixel dims (single mode: == dominant eye)
	float projc_left, projc_right, projc_top, projc_bottom;
	unsigned int canvas_w, canvas_h;

	// Auto pose-delay state: sliding-window vote over recent candidates
	int stab_auto_delay;
	uint8_t stab_auto_hist[STAB_AUTO_WIN];
	int stab_auto_pos, stab_auto_cnt;
	bool stab_time_reanchor; // absorb the pose-time jump after an adoption

	// Filter state
	bool stab_has_state;
	quatd q_smooth;
	quatd q_prev_raw; // One Euro speed estimate
	double omega_lp;  // One Euro speed estimate, low-passed (rad/s)
	double prev_pose_time;
	uint32_t stab_resync_count;

	// Positional compensation state (camera-frame correction, tangent units)
	vec3d p_smooth;
	vec3d stab_posc;      // current correction: d_cam / reference_depth
	vec3d stab_posc_last; // reused on motion-smoothed frames
	vec3d stab_p_raw;     // raw head position (telemetry)

	// Reprojection pass resources (Milestone B)
	ComPtr<ID3D11VertexShader> stabVS;
	ComPtr<ID3D11PixelShader> stabPS;
	ComPtr<ID3D11Buffer> stabCB;
	ComPtr<ID3D11SamplerState> stabSampler;
	ComPtr<ID3D11RasterizerState> stabRaster;
	ComPtr<ID3D11RenderTargetView> stabRTV;
	bool stab_shader_ok;
	bool stab_encode_srgb;
	bool stab_black_edges; // no crop margin: skip the clamp, show black borders
	quatd q_e2h;      // eye-to-head rotation, dominant eye (canted displays)
	quatd q_e2h2;     // eye-to-head rotation, other eye (wide)
	quatd q_err_last; // last applied correction, reused on motion-smoothed frames

	// Debug stats (2s window)
	double dbg_window_start;
	uint32_t dbg_window_frame_index;
	double dbg_max_dx, dbg_max_dy;
	uint32_t dbg_clamped;
	// Calibration score accumulator (Freeze View): stddev of the per-frame
	// correction DELTA - corr itself holds the accumulated offset when frozen,
	// its delta is what shake looks like.
	double dbg_cal_prev, dbg_cal_sum, dbg_cal_sum2;
	int dbg_cal_n;
};

// Compile the reprojection shaders and create the fixed GPU objects on the
// plugin's own device. Idempotent; returns false if anything fails.
static bool stab_create_gpu_resources(win_openvr *context)
{
	if (context->stabVS && context->stabPS && context->stabCB && context->stabSampler && context->stabRaster)
		return true;

	// Partial-failure retries must not overwrite live ComPtrs via GetAddressOf
	context->stabVS.Reset();
	context->stabPS.Reset();
	context->stabCB.Reset();
	context->stabSampler.Reset();
	context->stabRaster.Reset();

	const UINT flags = D3DCOMPILE_OPTIMIZATION_LEVEL3 | D3DCOMPILE_PACK_MATRIX_ROW_MAJOR;
	ComPtr<ID3DBlob> vsb, psb, errb;
	if (FAILED(D3DCompile(STAB_SHADER_SRC, strlen(STAB_SHADER_SRC), "stab", nullptr, nullptr, "VSMain",
			      "vs_5_0", flags, 0, vsb.GetAddressOf(), errb.GetAddressOf()))) {
		warn("stab: VS compile failed: %s", errb ? (const char *)errb->GetBufferPointer() : "(no log)");
		return false;
	}
	errb.Reset();
	if (FAILED(D3DCompile(STAB_SHADER_SRC, strlen(STAB_SHADER_SRC), "stab", nullptr, nullptr, "PSMain",
			      "ps_5_0", flags, 0, psb.GetAddressOf(), errb.GetAddressOf()))) {
		warn("stab: PS compile failed: %s", errb ? (const char *)errb->GetBufferPointer() : "(no log)");
		return false;
	}
	if (FAILED(context->shared_device->CreateVertexShader(vsb->GetBufferPointer(), vsb->GetBufferSize(), nullptr,
							      context->stabVS.GetAddressOf())) ||
	    FAILED(context->shared_device->CreatePixelShader(psb->GetBufferPointer(), psb->GetBufferSize(), nullptr,
							     context->stabPS.GetAddressOf()))) {
		warn("stab: shader object creation failed");
		return false;
	}

	D3D11_BUFFER_DESC bd = {};
	bd.ByteWidth = sizeof(StabCBData);
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	if (FAILED(context->shared_device->CreateBuffer(&bd, nullptr, context->stabCB.GetAddressOf()))) {
		warn("stab: constant buffer creation failed");
		return false;
	}

	D3D11_SAMPLER_DESC sd = {};
	sd.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	sd.AddressU = D3D11_TEXTURE_ADDRESS_BORDER;
	sd.AddressV = D3D11_TEXTURE_ADDRESS_BORDER;
	sd.AddressW = D3D11_TEXTURE_ADDRESS_BORDER;
	sd.BorderColor[3] = 1.0f; // opaque black outside the mirror frame
	sd.MaxLOD = D3D11_FLOAT32_MAX;
	if (FAILED(context->shared_device->CreateSamplerState(&sd, context->stabSampler.GetAddressOf()))) {
		warn("stab: sampler creation failed");
		return false;
	}

	D3D11_RASTERIZER_DESC rd = {};
	rd.FillMode = D3D11_FILL_SOLID;
	rd.CullMode = D3D11_CULL_NONE;
	rd.DepthClipEnable = TRUE;
	if (FAILED(context->shared_device->CreateRasterizerState(&rd, context->stabRaster.GetAddressOf()))) {
		warn("stab: rasterizer state creation failed");
		return false;
	}
	return true;
}

static bool stab_corners_ok(win_openvr *context, const quatd &q_err_head, const vec3d &posc_eye);
static bool stab_corners_feasible(win_openvr *context, const quatd &q_err_head, const vec3d &posc_head);

// Helper to destroy OBS texture
static void destroy_obs_texture(gs_texture_t **texture) {
	if (texture && *texture) {
		obs_enter_graphics();
		gs_texture_destroy(*texture);
		obs_leave_graphics();
		*texture = nullptr;
	}
}

/// This is the messiest code i have written in my life, one day i will fix it but that day is not today.
// Frame-boundary poller, defined below with the rest of the pairing code.
static void stab_poll_start(win_openvr *context);
static void stab_poll_stop(win_openvr *context);
static double stab_phase_of(win_openvr *context, uint32_t frame_index);

// Stack-only RAII for the GPU lock (the context struct stays POD; SRWLOCK's
// zero state is valid, so bzalloc initializes it). Defined before
// win_openvr_init, which is its first user.
struct StabGpuGuard {
	SRWLOCK *l;
	bool held;
	explicit StabGpuGuard(SRWLOCK *lk) : l(lk), held(true) { AcquireSRWLockExclusive(l); }
	void unlock()
	{
		if (held) {
			ReleaseSRWLockExclusive(l);
			held = false;
		}
	}
	~StabGpuGuard() { unlock(); }
};

static void win_openvr_init(void *data, bool forced = true)
{
	win_openvr *context = (win_openvr *)data;

	if (context->initialized || init_inprog) {
		return;
	}

	auto now = std::chrono::steady_clock::now();
	if (now - last_init_time < retry_delay) {
		return;
	}

	// A re-init after a previous success reaches here with the poller thread
	// still running; every init FAILURE path below calls VR_Shutdown, and
	// shutting the runtime down under a thread that is inside GetFrameTiming
	// is the crash class this plugin has hit before. Stop it first - it is
	// restarted on success.
	stab_poll_stop(context);
	last_init_time = now;

	init_inprog = true;

	vr::EVRInitError err = vr::VRInitError_None;
	vr::VR_Init(&err, vr::VRApplication_Background);
	if (err != vr::VRInitError_None) {
		warn("win_openvr_init: OpenVR initialization failed! %s", vr::VR_GetVRInitErrorAsEnglishDescription(err));
		init_inprog = false;
		return;
	}

	if (!context->shared_device.Get()) {
		HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, context->shared_device.GetAddressOf(), nullptr, context->shared_context.GetAddressOf());
		if (FAILED(hr)) {
			warn("win_openvr_init: SHARED D3D11CreateDevice failed");
			init_inprog = false;
			vr::VR_Shutdown();
			return;
		}
	}

	{
		// The render thread may be mid-warp on these exact pointers (init
		// can run on the UI thread via update/show); its D3D body holds the
		// same lock, so this mutation cannot interleave with a use. The
		// poller is already stopped (top of this function).
		StabGpuGuard g(&context->stab_gpu_lock);
		context->texCrop.Reset();
		context->tex.Reset();
		// Stale-session pipeline state drops INSIDE the lock too: between
		// texCrop's recreation below and the late resets further down, a
		// render tick could otherwise warp into the PREVIOUS session's RTV
		// or show its held pixels (panel F1).
		context->stabRTV.Reset();
		context->stab_shader_ok = false;
		context->holdSrv.Reset();
		context->holdSrv2.Reset();
		context->texHold.Reset();
		context->texHold2.Reset();
		context->hold_valid = false;
		// Abandon (do NOT Release) the previous mirror SRVs before re-acquiring:
		// the SteamVR runtime owns and caches them, and a plain Release()
		// mid-session corrupts its cache - crashed in d3d11.dll on the settings
		// re-init path (2026-07-18). Detach only nulls our pointer, so a failed
		// GetMirrorTextureD3D11 below still can't masquerade as success.
		context->mirrorSrv.Detach();
		context->mirrorSrv2.Detach();
		// Capture slots may hold a previous session's mirror size; drop them
		// so the desc-match rebuild starts clean.
		for (int s = 0; s < 2; s++)
			for (int e = 0; e < 2; e++) {
				if (context->cap_srv[s][e]) {
					context->cap_srv[s][e]->Release();
					context->cap_srv[s][e] = nullptr;
				}
				if (context->cap_tex[s][e]) {
					context->cap_tex[s][e]->Release();
					context->cap_tex[s][e] = nullptr;
				}
			}
		context->cap_valid[0] = context->cap_valid[1] = 0;
		context->stab_cap_due = 0;
	}

	IsVRSystemInitialized = true;

	if (!vr::VRCompositor()) {
		warn("win_openvr_show: VR Compositor not found");
		init_inprog = false;
		vr::VR_Shutdown();
		return;
	}

	vr::EVRCompositorError composError = vr::VRCompositor()->GetMirrorTextureD3D11(context->righteye ? vr::Eye_Right : vr::Eye_Left, context->shared_device.Get(), reinterpret_cast<void**>(context->mirrorSrv.GetAddressOf()));

	if (context->mirrorSrv) {
		context->mirrorSrv->GetResource(context->tex.GetAddressOf());
		if (context->tex) {
			D3D11_TEXTURE2D_DESC desc = {};
			ComPtr<ID3D11Texture2D> tex2D = nullptr;
			context->tex->QueryInterface(__uuidof(ID3D11Texture2D), reinterpret_cast<void**>(tex2D.GetAddressOf()));
			if (tex2D) {
				tex2D->GetDesc(&desc);
				context->device_width = desc.Width;
				context->device_height = desc.Height;

				vr::VRSystem()->GetProjectionRaw(context->righteye ? vr::Eye_Right : vr::Eye_Left,
								 &context->proj_left, &context->proj_right,
								 &context->proj_top, &context->proj_bottom);
				context->q_e2h = quat_normalize(quat_from_hmd34(vr::VRSystem()->GetEyeToHeadTransform(
					context->righteye ? vr::Eye_Right : vr::Eye_Left)));

				// Wide (both-eye) composite: acquire the other eye's mirror
				// and build a union-frustum canvas. Requires the shader pass.
				context->wide_active = false;
				context->canvas_w = context->device_width;
				context->canvas_h = context->device_height;
				context->projc_left = context->proj_left;
				context->projc_right = context->proj_right;
				context->projc_top = context->proj_top;
				context->projc_bottom = context->proj_bottom;
				if (context->eye_mode >= 2) {
					const vr::EVREye other = context->righteye ? vr::Eye_Left : vr::Eye_Right;
					if (!stab_create_gpu_resources(context)) {
						warn("wide: composite shader unavailable, using the dominant eye only");
					} else {
						vr::VRCompositor()->GetMirrorTextureD3D11(
							other, context->shared_device.Get(),
							reinterpret_cast<void **>(context->mirrorSrv2.GetAddressOf()));
						D3D11_SHADER_RESOURCE_VIEW_DESC sd1 = {}, sd2 = {};
						if (context->mirrorSrv2) {
							context->mirrorSrv->GetDesc(&sd1);
							context->mirrorSrv2->GetDesc(&sd2);
						}
						if (!context->mirrorSrv2) {
							warn("wide: second mirror texture unavailable, using the dominant eye only");
						} else if (sd1.Format != sd2.Format) {
							warn("wide: eye formats differ (%d vs %d), using the dominant eye only",
							     (int)sd1.Format, (int)sd2.Format);
							context->mirrorSrv2.Detach(); // runtime-owned, never Release
						} else {
							vr::VRSystem()->GetProjectionRaw(other, &context->proj2_left,
											 &context->proj2_right,
											 &context->proj2_top,
											 &context->proj2_bottom);
							context->q_e2h2 = quat_normalize(quat_from_hmd34(
								vr::VRSystem()->GetEyeToHeadTransform(other)));
							// Canvas rays live in HEAD space, so bound the canvas
							// by each eye's frustum corners projected through its
							// eye-to-head rotation (canted displays tilt the
							// coverage; raw eye-frame tangents would overhang it
							// and leave never-covered black bands). Horizontal
							// union; vertically each eye only guarantees its
							// shallower corners across its whole x-range, and the
							// canvas takes the intersection of those spans.
							// Identity e2h reduces to the plain union.
							double lC = 1e9, rC = -1e9, tC = -1e9, bC = 1e9;
							bool geom_ok = true;
							for (int e = 0; e < 2 && geom_ok; e++) {
								const quatd &qe = e ? context->q_e2h2 : context->q_e2h;
								const double le = e ? context->proj2_left : context->proj_left;
								const double re = e ? context->proj2_right : context->proj_right;
								const double te = e ? context->proj2_top : context->proj_top;
								const double be = e ? context->proj2_bottom : context->proj_bottom;
								double xtl, ytl, xtr, ytr, xbl, ybl, xbr, ybr;
								geom_ok = eye_corner_head_tangent(qe, le, te, &xtl, &ytl) &&
									  eye_corner_head_tangent(qe, re, te, &xtr, &ytr) &&
									  eye_corner_head_tangent(qe, le, be, &xbl, &ybl) &&
									  eye_corner_head_tangent(qe, re, be, &xbr, &ybr);
								if (!geom_ok)
									break;
								lC = std::min(lC, std::min(std::min(xtl, xtr), std::min(xbl, xbr)));
								rC = std::max(rC, std::max(std::max(xtl, xtr), std::max(xbl, xbr)));
								tC = std::max(tC, std::max(ytl, ytr));
								bC = std::min(bC, std::min(ybl, ybr));
							}
							if (!geom_ok || !(rC - lC > 0.1) || !(bC - tC > 0.1)) {
								warn("wide: degenerate combined frustum, using the dominant eye only");
								context->mirrorSrv2.Detach(); // runtime-owned, never Release
							} else {
								const double densx =
									context->device_width /
									((double)context->proj_right - context->proj_left);
								const double densy =
									context->device_height /
									((double)context->proj_bottom - context->proj_top);
								context->projc_left = (float)lC;
								context->projc_right = (float)rC;
								context->projc_top = (float)tC;
								context->projc_bottom = (float)bC;
								context->canvas_w = (unsigned int)std::lround(densx * (rC - lC));
								context->canvas_h = (unsigned int)std::lround(densy * (bC - tC));
								context->wide_active = true;
								info("wide: composite canvas %ux%u (mirror %ux%u per eye), dominant=%s",
								     context->canvas_w, context->canvas_h,
								     context->device_width, context->device_height,
								     context->righteye ? "right" : "left");
							}
						}
					}
				}
				const unsigned int base_w = context->canvas_w;
				const unsigned int base_h = context->canvas_h;

				// Pan and zoom
				int x = 0, y = 0;

				double scale_factor = context->scale_factor < 1.0 ? 1.0 : context->scale_factor;
				unsigned int scaled_width = static_cast<unsigned int>(base_w / scale_factor);
				unsigned int scaled_height = static_cast<unsigned int>(base_h / scale_factor);
				context->width = scaled_width;
				context->height = scaled_height;

				if (context->ar_crop) {
					double input_aspect_ratio = static_cast<double>(context->width) / context->height;
					double active_aspect_ratio = context->active_aspect_ratio;
					if (input_aspect_ratio > active_aspect_ratio) {
						context->width = static_cast<unsigned int>(context->height * active_aspect_ratio);
					} else if (input_aspect_ratio < active_aspect_ratio) {
						context->height = static_cast<unsigned int>(context->width / active_aspect_ratio);
					}
				}

				int x_offset = context->x_offset;
				int y_offset = context->y_offset;
				if (!context->righteye && !context->wide_active) {
					x_offset = -x_offset;
					x = base_w - scaled_width;
				}
				x += x_offset;
				y += y_offset;
				if (x + context->width > base_w) x = base_w - context->width;
				if (y + context->height > base_h) y = base_h - context->height;

				x = std::max(0, x);
				y = std::max(0, y);
				context->x = x;
				context->y = y;

				if (context->stabilize || context->wide_active) {
					// Stabilization steals margin from the zoom crop;
					// center the base crop so margin exists on all sides.
					int sx = (int)(base_w - context->width) / 2;
					int sy = (int)(base_h - context->height) / 2;
					sx += context->righteye ? context->x_offset : -context->x_offset;
					sy += context->y_offset;
					sx = std::min(std::max(0, sx), (int)(base_w - context->width));
					sy = std::min(std::max(0, sy), (int)(base_h - context->height));
					context->x = (unsigned int)sx;
					context->y = (unsigned int)sy;
				}

				context->stab_has_state = false;
				context->stab_lock_captured = false; // full-lock re-captures on re-init
				if (context->stabilize && (context->width == base_w ||
							   context->height == base_h)) {
					warn("stab: no crop margin (Zoom = 1) - corrections will reveal black edges; "
					     "increase Zoom (1.2+) to hide them");
				}
				if (context->stabilize && context->stab_debug) {
					info("stab: init eye=%s mirror=%ux%u canvas=%ux%u crop=%ux%u at (%u,%u), projraw l=%.3f r=%.3f t=%.3f b=%.3f",
					     context->righteye ? "right" : "left",
					     context->device_width, context->device_height,
					     base_w, base_h,
					     context->width, context->height, context->x, context->y,
					     context->proj_left, context->proj_right,
					     context->proj_top, context->proj_bottom);
				}

				desc.Width = context->width;
				desc.Height = context->height;
				desc.MipLevels = 1;
				desc.ArraySize = 1;
				desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
				desc.SampleDesc.Count = 1;
				desc.SampleDesc.Quality = 0;
				desc.Usage = D3D11_USAGE_DEFAULT;
				desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
				desc.CPUAccessFlags = 0;
				desc.MiscFlags = D3D11_RESOURCE_MISC_SHARED;

				HRESULT hr = context->shared_device->CreateTexture2D(&desc, nullptr, context->texCrop.GetAddressOf());
				if (FAILED(hr)) {
					warn("win_openvr_show: CreateTexture2D failed");
					init_inprog = false;
					vr::VR_Shutdown();
					tex2D.Reset();
					return;
				}

				HRESULT hrRes = context->texCrop->QueryInterface(__uuidof(IDXGIResource), reinterpret_cast<void**>(context->res.GetAddressOf()));
				if (FAILED(hrRes)) {
					warn("win_openvr_show: QueryInterface failed");
					init_inprog = false;
					tex2D.Reset();
					{
						// Locked: a graphics-thread render may hold a
						// live texCrop pointer (panel F2).
						StabGpuGuard g2(&context->stab_gpu_lock);
						context->texCrop.Reset();
					}
					vr::VR_Shutdown();
					return;
				}
				HANDLE handle = nullptr;
				HRESULT hrHandle = context->res->GetSharedHandle(&handle);
				if (FAILED(hrHandle)) {
					warn("win_openvr_show: GetSharedHandle failed");
					init_inprog = false;
					context->res.Reset();
					tex2D.Reset();
					{
						StabGpuGuard g2(&context->stab_gpu_lock); // panel F2
						context->texCrop.Reset();
					}
					vr::VR_Shutdown();
					return;
				}
				context->res.Reset();

				uint32_t GShandle = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(handle));
				destroy_obs_texture(&context->texture);
				obs_enter_graphics();
				context->texture = gs_texture_open_shared(GShandle);
				obs_leave_graphics();

				context->stabRTV.Reset();
				context->stab_shader_ok = false;
				if (context->stabilize || context->wide_active) {
					D3D11_SHADER_RESOURCE_VIEW_DESC srvd = {};
					context->mirrorSrv->GetDesc(&srvd);
					context->stab_encode_srgb =
						(srvd.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB ||
						 srvd.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB);
					context->q_err_last = {1.0, 0.0, 0.0, 0.0};
					bool warp_ok = stab_create_gpu_resources(context);
					if (warp_ok) {
						HRESULT hrRtv = context->shared_device->CreateRenderTargetView(
							context->texCrop.Get(), nullptr, context->stabRTV.GetAddressOf());
						if (FAILED(hrRtv)) {
							warn("stab: CreateRenderTargetView failed (0x%08lx)",
							     (unsigned long)hrRtv);
							warp_ok = false;
						}
					}
					if (!warp_ok && context->wide_active) {
						// texCrop and the shared OBS texture were already
						// created at CANVAS size; falling back to the copy
						// path here would leave a never-written strip drawn
						// every frame. Treat it like the other fatal init
						// failures and let the retry loop start over.
						warn("wide: render target creation failed, retrying init");
						destroy_obs_texture(&context->texture);
						{
							StabGpuGuard g2(&context->stab_gpu_lock); // panel F2
							context->stabRTV.Reset();
							context->texCrop.Reset();
						}
						context->wide_active = false;
						tex2D.Reset();
						init_inprog = false;
						vr::VR_Shutdown();
						return;
					}
					// Edge-pinned crop: corrections cannot stay inside the
					// frame, so run unclamped and let the border show black.
					context->stab_black_edges =
						warp_ok && !stab_corners_feasible(context, quatd{1.0, 0.0, 0.0, 0.0},
										  vec3d{0.0, 0.0, 0.0});
					context->stab_shader_ok = warp_ok;
					if (!warp_ok)
						warn("stab: reprojection pass unavailable, using crop-shift fallback");
					else if (context->stab_debug)
						info("stab: reprojection pass active, mirror_fmt=%d srgb_reencode=%d wide=%d",
						     (int)srvd.Format, (int)context->stab_encode_srgb,
						     (int)context->wide_active);
				}
				tex2D.Reset();
			}
		}
		context->initialized = true;
		context->lastFrame = 0;
		stab_poll_start(context);
		init_inprog = false;
	} else {
		warn("win_openvr_init: GetMirrorTextureD3D11 failed (%d)", (int)composError);
		init_inprog = false;
		vr::VR_Shutdown();
	}
}

static void win_openvr_init1(void *data, bool forced = true) {
	win_openvr *context = (win_openvr *)data;

	if (context->initialized || init_inprog) {
		return;
	}

	auto now = std::chrono::steady_clock::now();
	if (now - last_init_timeBUFFER < retry_delayBUFFER) {
		return;
	}
	last_init_timeBUFFER = now;

	win_openvr_init(data, forced);
}

static void win_openvr_deinit(void *data)
{
	win_openvr *context = (win_openvr *)data;

	// Stop the poller BEFORE VR_Shutdown - it calls into VRCompositor().
	stab_poll_stop(context);

	if (context->texture) destroy_obs_texture(&context->texture);
	{
		// Scoped: the guard MUST release before VR_Shutdown below (holding
		// the GPU lock across it is the documented deadlock class), and
		// destroy_obs_texture above must stay OUTSIDE it (obs_enter_graphics
		// vs this lock is an ABBA deadlock with the render thread).
		StabGpuGuard g(&context->stab_gpu_lock);
		if (context->shared_context)
			context->shared_context->Flush(); // drain any queued poller copy
		for (int s = 0; s < 2; s++)
			for (int e = 0; e < 2; e++) {
				if (context->cap_srv[s][e]) {
					context->cap_srv[s][e]->Release();
					context->cap_srv[s][e] = nullptr;
				}
				if (context->cap_tex[s][e]) {
					context->cap_tex[s][e]->Release();
					context->cap_tex[s][e] = nullptr;
				}
			}
		context->cap_valid[0] = context->cap_valid[1] = 0;
		context->stab_cap_due = 0;
		context->stabRTV.Reset();
		context->stabVS.Reset();
		context->stabPS.Reset();
		context->stabCB.Reset();
		context->stabSampler.Reset();
		context->stabRaster.Reset();
		context->stab_shader_ok = false;
		context->holdSrv.Reset();
		context->holdSrv2.Reset();
		context->texHold.Reset();
		context->texHold2.Reset();
		context->hold_valid = false;
		if (context->texCrop) context->texCrop.Reset();
		if (context->tex) context->tex.Reset();
		if (context->mirrorSrv) context->mirrorSrv.Reset();
		if (context->mirrorSrv2) context->mirrorSrv2.Reset();
		context->wide_active = false;
		if (context->res) context->res.Reset();
		if (context->shared_device) context->shared_device.Reset();
		if (context->shared_context) context->shared_context.Reset();
	}

	if (context->stab_telemetry_file) {
		fclose(context->stab_telemetry_file);
		context->stab_telemetry_file = nullptr;
	}

	vr::VR_Shutdown();

	context->initialized = false;
	init_inprog = false;
}

static const char *win_openvr_get_name(void *unused)
{
	return "OpenVR Capture";
}

static void win_openvr_update(void *data, obs_data_t *settings)
{
	struct win_openvr *context = (win_openvr *)data;

	// Eye selection; migrate the pre-wide "righteye" bool once
	int eye_mode;
	if (!obs_data_has_user_value(settings, "eye_mode") && obs_data_has_user_value(settings, "righteye")) {
		eye_mode = obs_data_get_bool(settings, "righteye") ? 1 : 0;
		obs_data_set_int(settings, "eye_mode", eye_mode);
	} else {
		eye_mode = (int)obs_data_get_int(settings, "eye_mode");
	}
	eye_mode = std::min(std::max(eye_mode, 0), 3);
	bool righteye = (eye_mode == 1 || eye_mode == 3); // dominant eye

	// zoom/scaling
	double scale_factor = obs_data_get_double(settings, "scale_factor");

	int x_offset = (int)obs_data_get_int(settings, "x_offset");
	int y_offset = (int)obs_data_get_int(settings, "y_offset");

	double active_aspect_ratio = obs_data_get_double(settings, "aspect_ratio");
	bool ar_crop = active_aspect_ratio != -1.0;

	if (active_aspect_ratio == 0.0) {
		int custom_width = (int)obs_data_get_int(settings, "custom_aspect_width");
		int custom_height = (int)obs_data_get_int(settings, "custom_aspect_height");
		if (custom_width > 0 && custom_height > 0) {
			active_aspect_ratio = static_cast<double>(custom_width) / custom_height;
		} else {
			active_aspect_ratio = 16.0 / 9.0;
		}
	}

	bool stabilize = obs_data_get_bool(settings, "stabilize");

	// Only crop geometry changes require re-acquiring the mirror texture; the
	// stabilization filter parameters below apply live.
	bool need_reinit = eye_mode != context->eye_mode ||
			   scale_factor != context->scale_factor ||
			   x_offset != context->x_offset ||
			   y_offset != context->y_offset ||
			   ar_crop != context->ar_crop ||
			   active_aspect_ratio != context->active_aspect_ratio ||
			   stabilize != context->stabilize;

	context->eye_mode = eye_mode;
	context->righteye = righteye;
	context->scale_factor = scale_factor;
	context->x_offset = x_offset;
	context->y_offset = y_offset;
	context->ar_crop = ar_crop;
	context->active_aspect_ratio = active_aspect_ratio;
	context->stabilize = stabilize;
	// One-time migration to the collapsed UI (schema 1): auto pairing, auto
	// lag and the one-frame hold become the only model. A Mirror Lag that was
	// hand-calibrated in the old manual mode seeds the new single slider;
	// retired keys get their user values cleared so the new defaults win.
	// Keys stay readable (scripts/websocket) - only the UI is gone.
	if ((int)obs_data_get_int(settings, "stab_schema") < 1) {
		if (obs_data_has_user_value(settings, "stab_pair_lag_ms") &&
		    !obs_data_has_user_value(settings, "stab_lag_base_ms") &&
		    obs_data_get_bool(settings, "stab_pair_auto") &&
		    !obs_data_get_bool(settings, "stab_lag_auto"))
			obs_data_set_double(settings, "stab_lag_base_ms",
					    obs_data_get_double(settings, "stab_pair_lag_ms"));
		obs_data_unset_user_value(settings, "stab_hold");
		obs_data_unset_user_value(settings, "stab_pair_auto");
		obs_data_unset_user_value(settings, "stab_lag_auto");
		obs_data_unset_user_value(settings, "stab_pair_phase");
		obs_data_set_int(settings, "stab_schema", 1);
	}

	int stab_pair_frames = std::min(std::max((int)obs_data_get_int(settings, "stab_pair_frames"), 0), 4);
	// Forced on since UI v1 (collapsed panel): time-based pairing is the only
	// model that survived the investigation. The settings keys are accepted
	// but no longer steer these.
	bool stab_pair_auto = true;
	double stab_pair_lag_ms = std::min(std::max(obs_data_get_double(settings, "stab_pair_lag_ms"), -20.0), 40.0);
	if (stab_pair_frames != context->stab_pair_frames || stab_pair_auto != context->stab_pair_auto ||
	    stab_pair_lag_ms != context->stab_pair_lag_ms) {
		// Any pairing change shifts the fetched pose timestamps by whole frames;
		// without a re-anchor an INCREASE gives dt <= 0 and the resync branch
		// blanks the correction for a frame.
		context->stab_time_reanchor = true;
		context->stab_has_sel = false;
		context->stab_pair_auto_cur = stab_pair_frames;
		context->stab_pair_auto_cand = -1;
		context->stab_pair_auto_hits = 0;
	}
	context->stab_pair_frames = stab_pair_frames;
	context->stab_pair_auto = stab_pair_auto;
	context->stab_pair_lag_ms = stab_pair_lag_ms;
	context->stab_pair_phase = true; // forced: poller phase always on (UI v1)
	bool stab_hold = obs_data_get_bool(settings, "stab_hold");
	if (stab_hold != context->stab_hold) {
		StabGpuGuard g(&context->stab_gpu_lock); // render may be mid-warp on these
		context->hold_valid = false;             // stale pixels must not be warped
		InterlockedExchange(&context->cap_valid[0], 0);
		InterlockedExchange(&context->cap_valid[1], 0);
		context->stab_time_reanchor = true;
	}
	context->stab_hold = stab_hold;
	InterlockedExchange(&context->stab_cap_on, stab_hold ? 1 : 0);
	bool stab_lag_auto = true; // forced: classifier is identity at n=1 (UI v1)
	double stab_lag_base_ms = std::min(std::max(obs_data_get_double(settings, "stab_lag_base_ms"), -20.0), 40.0);
	if (stab_lag_auto != context->stab_lag_auto || stab_lag_base_ms != context->stab_lag_base_ms) {
		context->stab_lag_n = 1; // re-classify from scratch
		context->stab_lag_dwell = 0;
		context->stab_time_reanchor = true;
	}
	context->stab_lag_auto = stab_lag_auto;
	context->stab_lag_base_ms = stab_lag_base_ms;

	int stab_smooth_ms;
	switch ((int)obs_data_get_int(settings, "stab_preset")) {
	case 1:
		stab_smooth_ms = 200; // Low
		break;
	case 2:
		stab_smooth_ms = 500; // Medium
		break;
	case 3:
		stab_smooth_ms = 1000; // High
		break;
	default:
		stab_smooth_ms = (int)obs_data_get_int(settings, "stab_smooth_ms"); // Custom
		break;
	}
	context->stab_tau = std::min(std::max(stab_smooth_ms, 50), 2000) / 1000.0;
	int stab_filter = (int)obs_data_get_int(settings, "stab_filter");
	if (stab_filter != context->stab_filter)
		context->stab_has_state = false; // start the speed estimate clean
	context->stab_filter = stab_filter;
	context->stab_roll_lock = obs_data_get_bool(settings, "stab_roll_lock");
	context->stab_horizon_lock = obs_data_get_bool(settings, "stab_horizon_lock");
	bool stab_full_lock = obs_data_get_bool(settings, "stab_full_lock");
	if (stab_full_lock != context->stab_full_lock)
		context->stab_lock_captured = false; // capture a fresh heading on each enable
	context->stab_full_lock = stab_full_lock;

	bool stab_pos_comp = obs_data_get_bool(settings, "stab_pos_comp");
	if (stab_pos_comp != context->stab_pos_comp)
		context->stab_has_state = false; // reseed the position state cleanly
	context->stab_pos_comp = stab_pos_comp;
	context->stab_pos_depth = std::min(std::max(obs_data_get_double(settings, "stab_pos_depth"), 0.3), 100.0);

	int stab_pose_delay = std::min(std::max((int)obs_data_get_int(settings, "stab_pose_delay"), -1), 15);
	if (stab_pose_delay != context->stab_pose_delay) {
		context->stab_auto_pos = 0;
		context->stab_auto_cnt = 0; // restart the vote window
		// A delay change shifts the fetched pose timestamps by whole frames;
		// without a re-anchor an INCREASE gives dt <= 0, and the resync
		// branch would snap the accumulated correction to zero for a frame.
		context->stab_time_reanchor = true;
		if (stab_pose_delay < 0)
			context->stab_auto_delay = 1; // fresh auto estimate from the sane default
	}
	context->stab_pose_delay = stab_pose_delay;
	context->stab_debug = obs_data_get_bool(settings, "stab_debug");
	context->stab_telemetry = obs_data_get_bool(settings, "stab_telemetry");

	// The warp loop closes the telemetry file when its checkbox goes off,
	// but stops running entirely when stabilization is disabled - close it
	// here too so the CSV tail is flushed as soon as recording stops.
	if ((!context->stabilize || !context->stab_telemetry) && context->stab_telemetry_file) {
		fclose(context->stab_telemetry_file);
		context->stab_telemetry_file = nullptr;
	}

	if (context->initialized && need_reinit) {
		context->initialized = false; // Force re-init
		win_openvr_init(data);
	}
}

static void win_openvr_defaults(obs_data_t *settings)
{
	obs_data_set_default_int(settings, "eye_mode", 1); // Right
	obs_data_set_default_double(settings, "aspect_ratio", -1.0);
	obs_data_set_default_int(settings, "custom_aspect_width", 16);
	obs_data_set_default_int(settings, "custom_aspect_height", 9);
	obs_data_set_default_double(settings, "scale_factor", 1.0);
	obs_data_set_default_int(settings, "x_offset", 0);
	obs_data_set_default_int(settings, "y_offset", 0);
	obs_data_set_default_bool(settings, "stabilize", false);
	obs_data_set_default_int(settings, "stab_preset", 2);
	obs_data_set_default_bool(settings, "stab_roll_lock", false);
	obs_data_set_default_bool(settings, "stab_horizon_lock", false);
	obs_data_set_default_bool(settings, "stab_full_lock", false);
	obs_data_set_default_bool(settings, "stab_pos_comp", false);
	obs_data_set_default_double(settings, "stab_pos_depth", 1.5);
	obs_data_set_default_int(settings, "stab_smooth_ms", 500);
	obs_data_set_default_int(settings, "stab_filter", 0);
	obs_data_set_default_int(settings, "stab_pair_frames", 1);
	obs_data_set_default_double(settings, "stab_pair_lag_ms", 12.5);
	obs_data_set_default_bool(settings, "stab_hold", true);
	obs_data_set_default_bool(settings, "stab_lag_auto", true);
	obs_data_set_default_bool(settings, "stab_pair_auto", true);
	obs_data_set_default_bool(settings, "stab_pair_phase", true);
	obs_data_set_default_bool(settings, "stab_advanced", false);
	obs_data_set_default_double(settings, "stab_lag_base_ms", 8.5);
	obs_data_set_default_int(settings, "stab_pose_delay", -1); // Auto (Tier-1 crop fallback)
	obs_data_set_default_bool(settings, "stab_debug", false);
	obs_data_set_default_bool(settings, "stab_telemetry", false);
}

static uint32_t win_openvr_getwidth(void *data)
{
	struct win_openvr *context = (win_openvr *)data;
	return context->width;
}

static uint32_t win_openvr_getheight(void *data)
{
	struct win_openvr *context = (win_openvr *)data;
	return context->height;
}

static void win_openvr_show(void *data)
{
	win_openvr_init1(data, true); // When showing do forced init without delay
}

static void win_openvr_hide(void *data)
{
	win_openvr_deinit(data);
}

static void *win_openvr_create(obs_data_t *settings, obs_source_t *source)
{
	struct win_openvr *context = (win_openvr *)bzalloc(sizeof(win_openvr));
	context->source = source;

	context->initialized = false;

	context->shared_device = nullptr;
	context->shared_context = nullptr;
	context->tex = nullptr;
	context->texture = nullptr;
	context->texCrop = nullptr;
	context->mirrorSrv = nullptr;

	context->width = context->height = 100;

	context->active_aspect_ratio = 16.0 / 9.0;
	context->stab_auto_delay = 1;

	win_openvr_update(context, settings);
	return context;
}

static void win_openvr_destroy(void *data)
{
	struct win_openvr *context = (win_openvr *)data;

	win_openvr_deinit(data);
	bfree(context);
}

// Which frames-ago timing entry does the mirror texture currently show?
// Auto mode: the newest record that has actually been presented. The correct
// offset is a property of the session (pipeline depth, refresh rate, motion
// smoothing), so a fixed manual value goes stale between VR launches.
// Call at most once per compositor frame (hysteresis counters advance).
static int stab_resolve_pose_delay(win_openvr *context)
{
	if (context->stab_pose_delay >= 0)
		return context->stab_pose_delay; // manual override

	int want = 1; // fallback when nothing is marked presented
	bool synth = false;
	for (uint32_t k = 0; k <= 3; k++) {
		vr::Compositor_FrameTiming t = {};
		t.m_nSize = sizeof(vr::Compositor_FrameTiming);
		if (!vr::VRCompositor()->GetFrameTiming(&t, k))
			break;
		if (t.m_nNumFramePresents > 0) {
			want = (int)k;
			synth = (t.m_nReprojectionFlags & vr::VRCompositor_ReprojectionMotion) != 0;
			break;
		}
	}

	// Sliding-window majority vote. Consecutive-streak hysteresis reset on
	// every noisy sample, so an alternating present pattern (dropped
	// frames, compositor/OBS rate mismatch) could pin Auto to a stale
	// value indefinitely; a windowed majority tolerates the noise.
	// Motion-smoothed frames don't vote - their present pattern is
	// synthetic while interpolation runs.
	if (!synth) {
		context->stab_auto_hist[context->stab_auto_pos] = (uint8_t)want;
		context->stab_auto_pos = (context->stab_auto_pos + 1) % STAB_AUTO_WIN;
		if (context->stab_auto_cnt < STAB_AUTO_WIN)
			context->stab_auto_cnt++;
	}
	if (context->stab_auto_cnt * 4 >= STAB_AUTO_WIN * 3) { // window ~3/4 full
		int counts[4] = {0, 0, 0, 0};
		for (int i = 0; i < context->stab_auto_cnt; i++)
			counts[context->stab_auto_hist[i] & 3]++;
		int best = 0;
		for (int k = 1; k < 4; k++)
			if (counts[k] > counts[best])
				best = k;
		// Adopt on a strong (70%) majority - or EVICT an incumbent that has
		// lost nearly all support: when the remaining votes split between
		// two neighbors (rate-mismatch beat pattern), neither reaches 70%,
		// and without eviction a value elected during a transient regime
		// (loading screen, half-rate stretch) would pin forever on zero
		// votes. Plurality (>=50%) takes over once the incumbent is <20%.
		const bool strong = counts[best] * 10 >= context->stab_auto_cnt * 7;
		const bool evict = counts[context->stab_auto_delay & 3] * 5 < context->stab_auto_cnt &&
				   counts[best] * 2 >= context->stab_auto_cnt;
		if (best != context->stab_auto_delay && (strong || evict)) {
			if (context->stab_debug)
				info("stab: auto pose delay %d -> %d (votes %d/%d/%d/%d)",
				     context->stab_auto_delay, best, counts[0], counts[1], counts[2],
				     counts[3]);
			context->stab_auto_delay = best;
			// Seamless switch: re-anchor the filter clock on the new
			// pose stream instead of reseeding - no correction snap.
			context->stab_time_reanchor = true;
		}
	}
	return context->stab_auto_delay;
}

// What the pairing actually landed on this frame. Every field describes the
// record whose pose was used, NOT the configured setting - the distinction was
// invisible until now and hid the silent fallback below.
struct StabPairInfo {
	int req_delay;      // offset asked for this frame (auto mode makes it vary)
	uint32_t dframe;    // records elapsed since our previous accepted sample
	// Sub-record sampling phase. At 120 records/s a record IS one vsync, so this
	// is exactly the phase that decides which frame the mirror holds - the term
	// no other field reports. May be unimplemented on a virtual display driver,
	// hence vsync_ok.
	int vsync_ok;
	double vsync_phase_ms;
	uint64_t vsync_counter;
	double phase_ms;   // measured by the boundary poller; <0 = unavailable
	double extrap_ms;  // forward pose extrapolation applied this frame
	double eff_lag_ms; // effective lag actually used (auto mode varies it)
	int lag_mode;      // classifier state: vsyncs per record (1 = native)
	int cap_used;      // 1 = warped a boundary-locked slot, 0 = fallback path
	double cap_phase_ms; // capture instant relative to its record boundary
	int pair_future;   // 1 = A0 guard fired: target past the newest ring stamp
	int eff_delay;      // cur - used frame index; differs on a fallback
	double age_ms;      // how far back in TIME the used record sits
	uint32_t presents;  // >1 would mean the scene texture was reused
	uint32_t dropped;   // additional times the previous frame was scanned out
	uint32_t mispres;
	uint32_t flags;     // raw m_nReprojectionFlags
	uint32_t predicted; // ADDITIONAL_PREDICTED_FRAMES - a regime label, NOT an offset term
	uint32_t vs_ready;
	uint32_t vs_view;
	float crender_start_ms; // compositor milestones, relative to the record's own
	float crender_gpu_ms;   // m_flSystemTimeInSeconds (openvr.h:2078-2084). Unlike
	float crender_cpu_ms;   // vs_view these come from the compositor's own render,
	float newframe_ready_ms; // so they may populate at idle where vs_view reads 0.
	float transfer_ms;
};

// Real refresh, cached. Converts the measured record period into "vsyncs per
// record" - the discriminator between native (1) and reprojected (2).
static double stab_vsync_period_s(win_openvr *context)
{
	// Expire the cache every ~10 s: the refresh can CHANGE mid-session
	// (90 -> 120 via SteamVR settings), and a stale V both mis-subtracts the
	// lag and parks the classifier ratio in its dead band indefinitely.
	if (context->stab_vsync_s > 0.0 && ++context->stab_vsync_age < 600)
		return context->stab_vsync_s;
	context->stab_vsync_age = 0;
	if (context->stab_vsync_wait > 0) {
		context->stab_vsync_wait--; // backoff instead of hammering a dead probe
		return context->stab_vsync_s;
	}
	if (!vr::VRSystem())
		return context->stab_vsync_s;
	vr::ETrackedPropertyError perr = vr::TrackedProp_Success;
	const float hz = vr::VRSystem()->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd,
								       vr::Prop_DisplayFrequency_Float, &perr);
	// The !(a && b) form also rejects NaN, which passes naive </> checks and
	// would be cached as a poisoned period forever.
	if (perr != vr::TrackedProp_Success || !(hz >= 30.0f && hz <= 1000.0f)) {
		if (!context->stab_vsync_warned) {
			context->stab_vsync_warned = true;
			warn("stab: display frequency unavailable (err=%d, hz=%f) - auto lag inert until it appears",
			     (int)perr, (double)hz);
		}
		context->stab_vsync_wait = 600;
		return context->stab_vsync_s; // keep a previously good value
	}
	const double v = 1.0 / (double)hz;
	if (fabs(v - context->stab_vsync_s) > 1e-6)
		info("stab: display %.2f Hz (vsync %.3f ms)", hz, 1000.0 / hz);
	context->stab_vsync_s = v;
	return v;
}

// Cadence tracker: measured record period + records-per-sample average. Runs on
// EVERY warp tick - this used to live inside stab_resolve_pair_frames, which the
// hold path bypasses, silently freezing both while holding.
static void stab_track_cadence(win_openvr *context, const vr::Compositor_FrameTiming *cur)
{
	const double t0 = cur->m_flSystemTimeInSeconds;
	const uint32_t dframe = cur->m_nFrameIndex - context->lastFrame;
	if (dframe > 0 && dframe < 16 && context->stab_last_t0 > 0.0) {
		const double per = (t0 - context->stab_last_t0) / (double)dframe;
		if (per > 0.002 && per < 0.05)
			context->stab_rec_period =
				context->stab_rec_period > 0.0
					? context->stab_rec_period + 0.05 * (per - context->stab_rec_period)
					: per;
	}
	context->stab_last_t0 = t0;
	context->stab_last_dframe = dframe;
	if (context->stab_dframe_avg <= 0.0)
		context->stab_dframe_avg = 1.0;
	context->stab_dframe_avg += 0.0165 * ((double)dframe - context->stab_dframe_avg);
}

// Effective mirror lag. Auto mode classifies the pipeline from the measured
// record period: records spanning n >= 2 vsyncs mean the app runs below
// refresh and async reprojection is filling the display - a stage that holds
// the mirror one record behind its timestamp. Measured: +8.5 ms native vs
// -8.5 reprojected, one 2-vsync record apart.
//
// Two hard-won rules shape this:
// 1. The subtraction is QUANTIZED to the vsync grid (n*V), never the raw
//    period average: game cadence contains 8-24% genuine 3-vsync gaps, so the
//    average sits 1.5-3.5 ms above the modal 2-vsync spacing and drifts with
//    load. Subtracting it would park the lag near a record boundary, where
//    per-tick selection ALTERNATES - and alternation reads as shake. The
//    user's calibrated optima match the modal spacing exactly.
// 2. A cadence change must persist ~2 s (dwell) before the mode flips: real
//    games burst to full rate for 0.4-2 s (menus, loading screens), and
//    riding those out consistently-slightly-wrong beats flapping through
//    them. Every accepted flip re-anchors: glide, not snap.
static double stab_effective_lag_s(win_openvr *context)
{
	if (!context->stab_lag_auto || !context->stab_pair_auto)
		return context->stab_pair_lag_ms * 0.001;
	double lag = context->stab_lag_base_ms * 0.001;
	const double V = stab_vsync_period_s(context);
	const double D = context->stab_rec_period;
	if (V > 0.0 && D > 0.0) {
		// Integer vsyncs-per-record with a +/-0.65 dead band around the
		// held value (subsumes the earlier 1.6x/1.4x hysteresis).
		const double r = D / V;
		int cand = context->stab_lag_n;
		if (r > (double)context->stab_lag_n + 0.65 || r < (double)context->stab_lag_n - 0.65)
			cand = (int)(r + 0.5);
		cand = std::min(std::max(cand, 1), 4);
		if (cand != context->stab_lag_n) {
			// Asymmetric dwell. Entering DEEPER reprojection flips fast
			// (~0.5 s): the native runs show zero reproj-direction bursts,
			// so a sustained rise is always a real load change - this is
			// what trims the session-start transient. Leaving reprojection
			// stays slow (~2.5 s): games genuinely burst to full rate for
			// 0.4-2 s in menus and loading screens, and flapping through
			// those is worse than riding them out.
			// Dwell counts RECORDS, not warp ticks, so the wall time is
			// rate-agnostic (ticks halve at canvas 30; records do not).
			const int need = (cand > context->stab_lag_n) ? 30 : 150;
			context->stab_lag_dwell += (int)(context->stab_last_dframe ? context->stab_last_dframe : 1);
			if (context->stab_lag_dwell >= need) {
				info("stab: auto lag %d -> %d vsyncs/record (record %.2f ms, vsync %.2f ms)",
				     context->stab_lag_n, cand, D * 1000.0, V * 1000.0);
				context->stab_lag_n = cand;
				context->stab_lag_dwell = 0;
				context->stab_time_reanchor = true;
			}
		} else {
			context->stab_lag_dwell = 0;
		}
	}
	if (context->stab_lag_n >= 2)
		lag -= (double)context->stab_lag_n * V;
	return lag;
}

// Boundary-count pairing: the mirror lag is one fixed DURATION L, and the offset
// is how many record boundaries fall within L of now:
//     offset = min{ k : (t_0 - t_k) >= L }
// The compositor sits on a hard ~120 Hz vsync grid and a record spans an INTEGER
// number of vsyncs - measured 1 at idle, 2 under game load - so the same physical
// L is a different record count per regime. That is precisely why a fixed integer
// cannot be right at both, and why sweeping the integer at idle found nothing.
// This is NOT the reverted wall-clock pairing: L is a lag, not an offset, and the
// pairing key stays the frame index - the duration only SELECTS among frame-indexed
// candidates using their own measured timestamps.
static int stab_resolve_pair_frames(win_openvr *context, const vr::Compositor_FrameTiming *cur, double L_s)
{
	if (!context->stab_pair_auto)
		return context->stab_pair_frames;
	double L = L_s;
	const double t0 = cur->m_flSystemTimeInSeconds;

	// Sub-record sampling phase. Measured: at idle our sampling is phase-LOCKED
	// to the record boundary (94% of samples inside an eighth of a period of it,
	// because 120 records/s against a ~60 Hz tick is an integer ratio), so the
	// selection sits permanently on the decision point and tick jitter flips it
	// every frame. That is the idle wobble. Folding the measured phase into the
	// window start moves the decision off the boundary and makes it per-sample
	// correct instead of a coin flip.
	bool phase_ok = false;
	if (context->stab_pair_phase) {
		const double phase = stab_phase_of(context, cur->m_nFrameIndex);
		if (phase >= 0.0) {
			L -= phase; // the sample instant sits phase into this record's life
			phase_ok = true;
		}
	}

	// A NEGATIVE effective lag means the mirror holds content newer than the
	// newest timing record - which is physically possible because we read the
	// timing on the CPU but the GPU samples the mirror some milliseconds later.
	// At 60 records/s that gap is a fraction of a frame; at 120 it spans a whole
	// one, so the correction arrives late and no backwards offset can fix it.
	// Extrapolate the pose forward instead of only selecting backwards.
	context->stab_extrap_s = (L < 0.0) ? -L : 0.0;

	int raw = 0;
	if (L > 0.0) {
		raw = 4;
		for (uint32_t k = 1; k <= 4; k++) {
			vr::Compositor_FrameTiming t = {};
			t.m_nSize = sizeof(vr::Compositor_FrameTiming);
			if (!vr::VRCompositor()->GetFrameTiming(&t, k)) {
				raw = (int)k - 1; // ran out of history
				break;
			}
			if (t0 - t.m_flSystemTimeInSeconds >= L) {
				raw = (int)k;
				break;
			}
		}
	}

	// Sampling-phase correction. dframe = compositor records elapsed since our
	// previous accepted sample, and it reads the phase directly: 1 means we
	// arrived early in the newest record's life and it is not in the mirror yet;
	// >= 2 means we are sampling at half the record rate, are LATE in that
	// record's life, and it already is. Measured: dframe is 2 on 96.3% of idle
	// samples and 1 on 75-91% in game - which is exactly why idle wants one less
	// than game, and why starting a recording changes the answer (it changes how
	// often video_render runs, hence how often we look).
	// Use a SLOW AVERAGE, not the instantaneous value: dframe >= 2 means two
	// different things. At idle it is the steady state (87% of samples) - we are
	// systematically undersampling and really are late. In game it is a one-off
	// hiccup (12.7%), and treating those as "late" flipped the offset 8 times in
	// 153 s, which is exactly the intermittent jump that was reported. Averaged
	// over ~1 s the two regimes separate cleanly: ~1.13 in game, ~1.89 at idle.
	// Cadence (record period + records-per-sample average) is tracked by
	// stab_track_cadence, which runs on every warp tick including hold.
	// The dframe heuristic was a stand-in for the sampling phase, back when we
	// could not measure it. The poller measures it directly now, so applying
	// both double-counts - it was silently subtracting 1 from the phase logic's
	// answer. Kept as the fallback for when the poller has no phase for this
	// record (measured 2.5-26% of frames depending on load).
	if (!context->stab_pair_phase || !phase_ok) {
		if (context->stab_dframe_avg >= 1.5 && raw > 0)
			raw--;
	}

	// Hysteresis. An offset that alternates between adjacent values is a
	// frame-to-frame discontinuity and reads as shake; being consistently a
	// little off just reads as slightly weaker stabilization. Consistency wins.
	if (raw == context->stab_pair_auto_cur) {
		context->stab_pair_auto_hits = 0;
		return raw;
	}
	if (raw != context->stab_pair_auto_cand) {
		context->stab_pair_auto_cand = raw;
		context->stab_pair_auto_hits = 1;
		return context->stab_pair_auto_cur;
	}
	if (++context->stab_pair_auto_hits < 3)
		return context->stab_pair_auto_cur;
	if (context->stab_debug)
		info("stab: pairing %d -> %d (dframe avg %.2f)", context->stab_pair_auto_cur, raw,
		     context->stab_dframe_avg);
	context->stab_pair_auto_cur = raw;
	return raw;
}

// Lazily (re)build one capture-slot texture to match the mirror. Recreates on
// any desc mismatch - a mirror-size change across a re-init would otherwise
// feed CopySubresourceRegion mismatched sizes, which D3D silently drops.
static bool stab_cap_ensure_eye(win_openvr *context, ID3D11ShaderResourceView *mirror, ID3D11Texture2D **tex,
				ID3D11ShaderResourceView **srv)
{
	if (!mirror)
		return false;
	Microsoft::WRL::ComPtr<ID3D11Resource> res;
	mirror->GetResource(&res);
	Microsoft::WRL::ComPtr<ID3D11Texture2D> src;
	if (FAILED(res.As(&src)))
		return false;
	D3D11_TEXTURE2D_DESC want = {};
	src->GetDesc(&want);
	if (*tex) {
		D3D11_TEXTURE2D_DESC have = {};
		(*tex)->GetDesc(&have);
		if (have.Width == want.Width && have.Height == want.Height && have.Format == want.Format)
			return true;
		(*srv)->Release();
		(*tex)->Release();
		*srv = nullptr;
		*tex = nullptr;
	}
	D3D11_TEXTURE2D_DESC d = want;
	d.MipLevels = 1;
	d.ArraySize = 1;
	d.Usage = D3D11_USAGE_DEFAULT;
	d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	d.CPUAccessFlags = 0;
	d.MiscFlags = 0;
	if (FAILED(context->shared_device->CreateTexture2D(&d, nullptr, tex)))
		return false;
	D3D11_SHADER_RESOURCE_VIEW_DESC msd = {};
	mirror->GetDesc(&msd);
	D3D11_SHADER_RESOURCE_VIEW_DESC sd = {};
	sd.Format = msd.Format; // keep the mirror's format: sRGB handling identical
	sd.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	sd.Texture2D.MipLevels = 1;
	if (FAILED(context->shared_device->CreateShaderResourceView(*tex, &sd, srv))) {
		(*tex)->Release();
		*tex = nullptr;
		return false;
	}
	return true;
}

// Poller-side mid-slot capture. Stamp BEFORE the lock (the stamp names the
// instant; only the copy waits). Flush is MANDATORY: unflushed, the copy sits
// queued until the next render tick and the stamp lies by up to a canvas
// period - the exact race this thread exists to kill.
static void stab_cap_execute(win_openvr *context, int64_t due)
{
	StabGpuGuard g(&context->stab_gpu_lock);
	// Stamp INSIDE the lock (panel F3): a pre-lock stamp lies about the pixels
	// whenever the lock wait crosses a mirror update. And if we arrived late -
	// bad wake, long wait - skip publishing rather than pair near-next-update
	// pixels with the nominal instant: the previous slot stays valid, which is
	// consistently one-older, never alternating.
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	if (context->stab_qpc_freq > 0 && (double)(now.QuadPart - due) / (double)context->stab_qpc_freq > 0.0025)
		return;
	if (InterlockedCompareExchange(&context->stab_poll_run, 1, 1) == 0)
		return; // shutting down
	if (!context->shared_context || !context->mirrorSrv)
		return;
	if (context->wide_active && !context->mirrorSrv2)
		return;
	const int slot = 1 - context->cap_newest;
	if (!stab_cap_ensure_eye(context, context->mirrorSrv.Get(), &context->cap_tex[slot][0],
				 &context->cap_srv[slot][0]))
		return;
	if (context->wide_active &&
	    !stab_cap_ensure_eye(context, context->mirrorSrv2.Get(), &context->cap_tex[slot][1],
				 &context->cap_srv[slot][1]))
		return;
	InterlockedExchange(&context->cap_valid[slot], 0);
	Microsoft::WRL::ComPtr<ID3D11Resource> src;
	context->mirrorSrv->GetResource(&src);
	context->shared_context->CopySubresourceRegion(context->cap_tex[slot][0], 0, 0, 0, 0, src.Get(), 0, nullptr);
	if (context->wide_active) {
		Microsoft::WRL::ComPtr<ID3D11Resource> src2;
		context->mirrorSrv2->GetResource(&src2);
		context->shared_context->CopySubresourceRegion(context->cap_tex[slot][1], 0, 0, 0, 0, src2.Get(), 0,
							       nullptr);
	}
	context->shared_context->Flush();
	const int pos = (int)context->stab_poll_pos;
	// Pair against the NOMINAL capture instant (boundary stamp + V/2), not the
	// actual wake: removes the 0-1.6 ms wake-quantization term from the lookup
	// margin (panel model-transfer guard #1). The late-skip above bounds how
	// far nominal and actual can diverge.
	context->cap_qpc[slot] = due;
	context->cap_idx[slot] = context->stab_poll_idx[pos];
	// ACTUAL phase relative to the record boundary - the race telemetry.
	// Drifting toward ~0.5 ms in native mode means racing the mirror update.
	context->cap_phase_ms[slot] =
		(double)(now.QuadPart - context->stab_poll_qpc[pos]) * 1000.0 / (double)context->stab_qpc_freq;
	context->cap_newest = slot;
	InterlockedExchange(&context->cap_valid[slot], 1);
}

// Poller: watch the compositor's frame index far faster than we render, and
// stamp QPC at every boundary. GetFrameTiming is a read out of the runtime's
// shared memory, so this is cheap; it never touches D3D or the mirror SRV.
static DWORD WINAPI stab_poll_proc(LPVOID param)
{
	win_openvr *context = (win_openvr *)param;
	uint32_t last = 0;
	bool have = false;
	while (InterlockedCompareExchange(&context->stab_poll_run, 1, 1) != 0) {
		LARGE_INTEGER wake;
		QueryPerformanceCounter(&wake);
		// Self-measure wake cadence; degrade to render-tick capture rather
		// than mispair quietly if Sleep(1) stops being ~1 kHz.
		double this_gap_ms = 0.0;
		if (context->poll_prev_wake) {
			const double gap_ms =
				(double)(wake.QuadPart - context->poll_prev_wake) * 1000.0 / (double)context->stab_qpc_freq;
			this_gap_ms = gap_ms;
			context->poll_seen++;
			if (gap_ms > 3.0)
				context->poll_slow++;
			if (context->poll_seen >= 1000) {
				const bool bad = context->poll_slow * 20 >= context->poll_seen; // >5% slow
				if (bad != context->poll_degraded) {
					context->poll_degraded = bad;
					warn("stab: poller wake cadence %s - %s capture",
					     bad ? "degraded (>3 ms)" : "recovered",
					     bad ? "reverting to render-tick" : "resuming boundary-locked");
				}
				context->poll_slow = 0;
				context->poll_seen = 0;
			}
		}
		context->poll_prev_wake = wake.QuadPart;

		vr::Compositor_FrameTiming t = {};
		t.m_nSize = sizeof(vr::Compositor_FrameTiming);
		if (vr::VRCompositor() && vr::VRCompositor()->GetFrameTiming(&t, 0)) {
			if (!have || t.m_nFrameIndex != last) {
				const int slot = (int)((context->stab_poll_pos + 1) & 15);
				context->stab_poll_idx[slot] = t.m_nFrameIndex;
				context->stab_poll_qpc[slot] = wake.QuadPart;
				// publish last: readers key off pos
				InterlockedExchange(&context->stab_poll_pos, slot);
				last = t.m_nFrameIndex;
				have = true;
				// Schedule this record's mid-slot capture. A wake that was
				// itself late (>3 ms gap) detected this boundary late too -
				// its stamp is already suspect, so sit this record out
				// rather than capture off a lying reference (panel F4).
				if (InterlockedCompareExchange(&context->stab_cap_on, 1, 1) != 0 &&
				    !context->poll_degraded && this_gap_ms <= 3.0) {
					const double V =
						context->stab_vsync_s > 0.0 ? context->stab_vsync_s : 0.00833;
					context->stab_cap_due =
						wake.QuadPart + (int64_t)(0.5 * V * (double)context->stab_qpc_freq);
				}
			}
		}
		if (context->stab_cap_due) {
			LARGE_INTEGER now2;
			QueryPerformanceCounter(&now2);
			if (now2.QuadPart >= context->stab_cap_due) {
				const int64_t due = context->stab_cap_due;
				context->stab_cap_due = 0;
				stab_cap_execute(context, due);
			}
		}
		Sleep(1); // ~1 kHz; a boundary is never missed by more than a ms
	}
	return 0;
}

static void stab_poll_start(win_openvr *context)
{
	if (context->stab_poll_thread)
		return;
	LARGE_INTEGER f;
	QueryPerformanceFrequency(&f);
	context->stab_qpc_freq = f.QuadPart;
	context->stab_poll_pos = 0;
	// Clear the ring: after a SteamVR restart the frame indices start over low,
	// and stale stamps from the dead session would otherwise win QPC lookups.
	memset((void *)context->stab_poll_idx, 0, sizeof(context->stab_poll_idx));
	memset((void *)context->stab_poll_qpc, 0, sizeof(context->stab_poll_qpc));
	InterlockedExchange(&context->stab_poll_run, 1);
	context->stab_poll_thread = CreateThread(nullptr, 0, stab_poll_proc, context, 0, nullptr);
	if (!context->stab_poll_thread)
		warn("stab: could not start the frame-boundary poller; phase will be unavailable");
}

static void stab_poll_stop(win_openvr *context)
{
	if (!context->stab_poll_thread)
		return;
	InterlockedExchange(&context->stab_poll_run, 0);
	// INFINITE join: proceeding past a live poller frees D3D resources under
	// it. The poller's only blocking points are bounded (the SRW lock, held
	// microseconds by others, and Sleep(1)) - and no caller of this function
	// may hold the GPU lock, or this join self-deadlocks.
	WaitForSingleObject(context->stab_poll_thread, INFINITE);
	CloseHandle(context->stab_poll_thread);
	context->stab_poll_thread = nullptr;
}

// Seconds since the record with this frame index began, measured. Returns <0 if
// the poller has not seen that record (it may be newer than the last boundary).
static double stab_phase_of(win_openvr *context, uint32_t frame_index)
{
	if (!context->stab_poll_thread || context->stab_qpc_freq <= 0)
		return -1.0;
	const int pos = (int)context->stab_poll_pos;
	for (int i = 0; i < 16; i++) {
		const int slot = (pos - i) & 15;
		if (context->stab_poll_idx[slot] == frame_index) {
			LARGE_INTEGER now;
			QueryPerformanceCounter(&now);
			const double dt =
				(double)(now.QuadPart - context->stab_poll_qpc[slot]) / (double)context->stab_qpc_freq;
			return (dt >= 0.0 && dt < 0.5) ? dt : -1.0;
		}
	}
	return -1.0;
}

// Which record was newest at a given QPC instant, from the poller's boundary
// stamps. Returns 0 if the instant is outside the ring's history.
static uint32_t stab_record_at_qpc(win_openvr *context, int64_t target)
{
	const int pos = (int)context->stab_poll_pos;
	uint32_t best = 0;
	int64_t bestq = 0;
	bool found = false;
	for (int i = 0; i < 16; i++) {
		const int slot = (pos - i) & 15;
		const int64_t q = context->stab_poll_qpc[slot];
		if (q == 0)
			continue;
		if (q <= target && (!found || q > bestq)) {
			bestq = q;
			best = context->stab_poll_idx[slot];
			found = true;
		}
	}
	return found ? best : 0;
}

// Lazily build a private copy target matching the mirror. Copying FROM the
// mirror's underlying resource is fine - it is a normal owned ref - but the
// mirror SRV itself must never be Released mid-session.
static bool stab_hold_ensure(win_openvr *context, ID3D11ShaderResourceView *srv,
			     Microsoft::WRL::ComPtr<ID3D11Texture2D> &tex,
			     Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> &outSrv)
{
	if (!srv || !context->shared_device)
		return false;
	Microsoft::WRL::ComPtr<ID3D11Resource> res;
	srv->GetResource(&res);
	Microsoft::WRL::ComPtr<ID3D11Texture2D> src;
	if (FAILED(res.As(&src)))
		return false;
	D3D11_TEXTURE2D_DESC d = {};
	src->GetDesc(&d);
	// Desc-match rebuild (panel F1): a mirror-size change across a re-init
	// would otherwise feed CopySubresourceRegion mismatched sizes, which D3D
	// silently drops - permanently frozen held pixels.
	if (tex && outSrv) {
		D3D11_TEXTURE2D_DESC have = {};
		tex->GetDesc(&have);
		if (have.Width == d.Width && have.Height == d.Height && have.Format == d.Format)
			return true;
		outSrv.Reset();
		tex.Reset();
	}
	d.MipLevels = 1;
	d.ArraySize = 1;
	d.Usage = D3D11_USAGE_DEFAULT;
	d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	d.CPUAccessFlags = 0;
	d.MiscFlags = 0;
	if (FAILED(context->shared_device->CreateTexture2D(&d, nullptr, tex.GetAddressOf())))
		return false;
	D3D11_SHADER_RESOURCE_VIEW_DESC msd = {};
	srv->GetDesc(&msd);
	D3D11_SHADER_RESOURCE_VIEW_DESC sd = {};
	sd.Format = msd.Format; // keep the mirror's format so sRGB handling is identical
	sd.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	sd.Texture2D.MipLevels = 1;
	if (FAILED(context->shared_device->CreateShaderResourceView(tex.Get(), &sd, outSrv.GetAddressOf()))) {
		tex.Reset();
		return false;
	}
	info("stab: one-frame hold buffer %ux%u fmt=%d", d.Width, d.Height, (int)d.Format);
	return true;
}

// Copy the live mirror into the hold buffer for the NEXT tick to warp.
static void stab_hold_capture(win_openvr *context, ID3D11DeviceContext *ctx,
			      const vr::Compositor_FrameTiming *cur)
{
	if (!stab_hold_ensure(context, context->mirrorSrv.Get(), context->texHold, context->holdSrv))
		return;
	if (context->wide_active &&
	    !stab_hold_ensure(context, context->mirrorSrv2.Get(), context->texHold2, context->holdSrv2))
		return;
	Microsoft::WRL::ComPtr<ID3D11Resource> src;
	context->mirrorSrv->GetResource(&src);
	ctx->CopySubresourceRegion(context->texHold.Get(), 0, 0, 0, 0, src.Get(), 0, nullptr);
	if (context->wide_active && context->mirrorSrv2) {
		Microsoft::WRL::ComPtr<ID3D11Resource> src2;
		context->mirrorSrv2->GetResource(&src2);
		ctx->CopySubresourceRegion(context->texHold2.Get(), 0, 0, 0, 0, src2.Get(), 0, nullptr);
	}
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	context->hold_qpc = now.QuadPart;
	context->hold_frame = cur->m_nFrameIndex;
	context->hold_valid = true;
}

// Fetch the raw pose + reprojection flag matching the current mirror frame.
static bool stab_fetch_pose(const vr::Compositor_FrameTiming *cur, int pose_delay, quatd *q_out, vec3d *p_out,
			    double *t_out, bool *frozen_out, StabPairInfo *info_out)
{
	// The newest timing entry is the frame the compositor just started; the
	// mirror shows an older, already-composited frame. Track WHICH record we
	// settled on - on a failed fetch or invalid pose we fall back to cur, which
	// silently collapses the offset to zero.
	const vr::Compositor_FrameTiming *used = cur;
	vr::Compositor_FrameTiming past = {};
	if (pose_delay > 0) {
		past.m_nSize = sizeof(vr::Compositor_FrameTiming);
		if (vr::VRCompositor()->GetFrameTiming(&past, (uint32_t)pose_delay) &&
		    past.m_HmdPose.bPoseIsValid)
			used = &past;
	}
	const vr::TrackedDevicePose_t *pose = &used->m_HmdPose;

	if (info_out) {
		info_out->req_delay = pose_delay;
		info_out->eff_delay = (int)(cur->m_nFrameIndex - used->m_nFrameIndex);
		info_out->age_ms = (cur->m_flSystemTimeInSeconds - used->m_flSystemTimeInSeconds) * 1000.0;
		info_out->presents = used->m_nNumFramePresents;
		info_out->dropped = used->m_nNumDroppedFrames;
		info_out->mispres = used->m_nNumMisPresented;
		info_out->flags = used->m_nReprojectionFlags;
		info_out->predicted = VR_COMPOSITOR_ADDITIONAL_PREDICTED_FRAMES(*used);
		info_out->vs_ready = used->m_nNumVSyncsReadyForUse;
		info_out->vs_view = used->m_nNumVSyncsToFirstView;
		info_out->crender_start_ms = used->m_flCompositorRenderStartMs;
		info_out->crender_gpu_ms = used->m_flCompositorRenderGpuMs;
		info_out->crender_cpu_ms = used->m_flCompositorRenderCpuMs;
		info_out->newframe_ready_ms = used->m_flNewFrameReadyMs;
		info_out->transfer_ms = used->m_flTransferLatencyMs;
	}

	if (!pose->bPoseIsValid || pose->eTrackingResult != vr::TrackingResult_Running_OK)
		return false;

	*q_out = quat_normalize(quat_from_hmd34(pose->mDeviceToAbsoluteTracking));
	const vr::HmdMatrix34_t &hm = pose->mDeviceToAbsoluteTracking;
	*p_out = {hm.m[0][3], hm.m[1][3], hm.m[2][3]};
	*t_out = used->m_flSystemTimeInSeconds;
	*frozen_out = (used->m_nReprojectionFlags & vr::VRCompositor_ReprojectionMotion) != 0;
	return true;
}

// Resolve the smoothing/lock target for a pose. Priority: Full Lock (debug)
// freezes ALL axes to a heading captured when it was enabled, so any head
// motion should render a frozen image - the definitive all-axis warp check.
// Otherwise Horizon Lock zeroes pitch+roll, Roll Lock zeroes roll, else the
// pose passes through (normal smoothing chases the raw/averaged pose).
static quatd stab_lock_target(win_openvr *context, const quatd &q_pose)
{
	if (context->stab_full_lock) {
		if (!context->stab_lock_captured) {
			// Capture the pose EXACTLY (no leveling): zero correction at
			// the moment of enable, so only subsequent motion exercises
			// the warp - enabling while tilted must not inject a jump.
			context->stab_lock_ref = quat_normalize(q_pose);
			context->stab_lock_captured = true;
		}
		return context->stab_lock_ref;
	}
	context->stab_lock_captured = false; // re-capture next time it is enabled
	if (context->stab_horizon_lock)
		return quat_level_horizon(q_pose);
	if (context->stab_roll_lock)
		return quat_level_roll(q_pose);
	return q_pose;
}

// Filter core: advance the damped orientation average and the positional
// compensation from the raw pose of the displayed frame. Returns false when
// no correction should be applied this frame (seed frame, timing resync).
static bool stab_filter_core(win_openvr *context, const vr::Compositor_FrameTiming *cur, const quatd &q_a,
			     const vec3d &p_a, double pose_time)
{
	const double kPi = 3.14159265358979323846;

	if (!context->stab_has_state) {
		context->q_smooth = q_a;
		context->q_prev_raw = q_a;
		context->omega_lp = 0.0;
		context->p_smooth = p_a;
		context->stab_q_ref = q_a;
		context->stab_p_raw = p_a;
		context->stab_posc = {0.0, 0.0, 0.0};
		context->stab_posc_last = {0.0, 0.0, 0.0};
		context->prev_pose_time = pose_time;
		context->stab_resync_count = 0;
		context->dbg_window_start = pose_time;
		context->dbg_window_frame_index = cur->m_nFrameIndex;
		context->dbg_max_dx = context->dbg_max_dy = 0.0;
		context->dbg_clamped = 0;
		context->stab_time_reanchor = false;
		context->stab_has_state = true;
		return false;
	}

	if (context->stab_time_reanchor) {
		// Auto pose delay switched streams: absorb the timestamp jump and
		// hold this frame's smoothing step; q_smooth stays continuous, so
		// the applied correction glides instead of snapping.
		context->stab_time_reanchor = false;
		context->prev_pose_time = pose_time;
		context->stab_q_ref = q_a;
		context->stab_p_raw = p_a;
		return true;
	}

	const double dt = pose_time - context->prev_pose_time;
	context->prev_pose_time = pose_time;
	if (dt <= 0.0 || dt > 0.25) {
		// Full Lock holds its captured heading across a timing hitch instead
		// of snapping the smoothed pose to the raw one (which would briefly
		// unfreeze the image).
		context->q_smooth = (context->stab_full_lock && context->stab_lock_captured)
					    ? context->stab_lock_ref
					    : q_a;
		context->q_prev_raw = q_a;
		context->omega_lp = 0.0;
		context->p_smooth = p_a;
		context->stab_posc = {0.0, 0.0, 0.0};
		context->stab_q_ref = q_a;
		context->stab_p_raw = p_a;
		context->dbg_window_start = pose_time;
		context->dbg_window_frame_index = cur->m_nFrameIndex;
		if (++context->stab_resync_count == 200)
			warn("stab: pose timestamps not advancing sanely (dt=%f), stabilization is inactive", dt);
		return false;
	}
	context->stab_resync_count = 0;

	// Both filters share the Smoothing Time knob as the rest-state time
	// constant, so they behave identically on a still head and differ only
	// in motion - a clean A/B comparison.
	// - Damped average: fixed cutoff; suppression is the same at rest and
	//   during active motion, deliberate turns trail by up to tau * rate.
	// - One Euro: cutoff rises with head speed (beta term); turns trail
	//   far less, but fast motion also lets more tremor through.
	double alpha;
	if (context->stab_filter == 1) {
		double d_raw = fabs(quat_dot(context->q_prev_raw, q_a));
		if (d_raw > 1.0)
			d_raw = 1.0;
		const double omega = 2.0 * acos(d_raw) / dt; // rad/s
		context->q_prev_raw = q_a;
		const double a_d = 1.0 - exp(-2.0 * kPi * 1.0 * dt); // 1 Hz derivative low-pass
		context->omega_lp += a_d * (omega - context->omega_lp);
		const double fc_min = 1.0 / (2.0 * kPi * context->stab_tau);
		const double beta = 0.7;
		const double fc = fc_min + beta * context->omega_lp;
		alpha = 1.0 - exp(-2.0 * kPi * fc * dt);
	} else {
		alpha = 1.0 - exp(-dt / context->stab_tau);
	}
	// With roll lock, the filter chases the LEVELED pose: steady state is an
	// exactly level horizon, and toggling the lock glides at the filter rate.
	const quatd q_target = stab_lock_target(context, q_a);
	context->q_smooth = quat_normalize(quat_slerp(context->q_smooth, q_target, alpha));

	// Hard 45-deg trail cap. Unlike the corner clamp (skipped in black-edges
	// mode), this always holds: without it a fast sustained spin winds the
	// error toward 180 deg, where slerp's hemisphere flip chases the target
	// BACKWARD and the warp renders garbage. At the cap the view rides a
	// fixed offset behind the turn and glides back when it ends.
	const double cap = 45.0 * (kPi / 180.0);
	double d_err = fabs(quat_dot(context->q_smooth, q_a));
	if (d_err > 1.0)
		d_err = 1.0;
	const double err = 2.0 * acos(d_err);
	if (err > cap)
		context->q_smooth = quat_normalize(quat_slerp(q_a, context->q_smooth, cap / err));

	// Positional jitter compensation: low-pass the head position and correct
	// the (displayed-raw - smoothed) delta at an assumed scene depth. Capped
	// at 2 cm so deliberate locomotion passes through untouched.
	context->stab_p_raw = p_a;
	context->stab_q_ref = q_a;
	if (context->stab_pos_comp) {
		const double a_p = 1.0 - exp(-2.0 * kPi * 0.75 * dt); // 0.75 Hz position low-pass
		context->p_smooth = {context->p_smooth.x + a_p * (p_a.x - context->p_smooth.x),
				     context->p_smooth.y + a_p * (p_a.y - context->p_smooth.y),
				     context->p_smooth.z + a_p * (p_a.z - context->p_smooth.z)};
		const vec3d dw = {p_a.x - context->p_smooth.x, p_a.y - context->p_smooth.y,
				  p_a.z - context->p_smooth.z};
		vec3d dc = quat_rotate(quat_conj(q_a), dw); // camera-frame delta, meters
		const double len = sqrt(dc.x * dc.x + dc.y * dc.y + dc.z * dc.z);
		const double cap = 0.02;
		if (len > cap) {
			const double s = cap / len;
			dc = {dc.x * s, dc.y * s, dc.z * s};
			const vec3d dwc = quat_rotate(q_a, dc);
			context->p_smooth = {p_a.x - dwc.x, p_a.y - dwc.y, p_a.z - dwc.z};
		}
		vec3d pc = {dc.x / context->stab_pos_depth, dc.y / context->stab_pos_depth,
			    dc.z / context->stab_pos_depth};
		if (!std::isfinite(pc.x) || !std::isfinite(pc.y) || !std::isfinite(pc.z)) {
			pc = {0.0, 0.0, 0.0};
			context->p_smooth = p_a; // self-heal once poses are finite again
		}
		context->stab_posc = pc;
	} else {
		context->p_smooth = p_a;
		context->stab_posc = {0.0, 0.0, 0.0};
	}

	return true;
}

// Fetch the raw pose and run the filter on it.
static bool stab_update_filter(win_openvr *context, const vr::Compositor_FrameTiming *cur, int pose_delay,
			       quatd *q_a_out)
{
	quatd q;
	vec3d p;
	double t;
	bool frozen;
	if (!stab_fetch_pose(cur, pose_delay, &q, &p, &t, &frozen, nullptr)) {
		context->stab_has_state = false;
		return false;
	}
	*q_a_out = q;
	return stab_filter_core(context, cur, q, p, t);
}

// Debug telemetry shared by both stabilization paths; logs every ~2 s.
static void stab_debug_stats(win_openvr *context, const vr::Compositor_FrameTiming *cur, double corr_a,
			     double corr_b, bool clamped, const char *unit)
{
	// Calibration score runs whenever Freeze View is on, independent of Debug
	// Logging - it IS the calibration instrument: sweep Mirror Lag until the
	// printed number bottoms out. No VR legs required.
	if (context->stab_full_lock) {
		if (context->dbg_cal_n > 0) {
			const double d = corr_a - context->dbg_cal_prev;
			context->dbg_cal_sum += d;
			context->dbg_cal_sum2 += d * d;
		}
		context->dbg_cal_prev = corr_a;
		context->dbg_cal_n++;
	} else {
		context->dbg_cal_n = 0;
		context->dbg_cal_sum = context->dbg_cal_sum2 = 0.0;
	}
	if (!context->stab_debug && !context->stab_full_lock)
		return;
	if (fabs(corr_a) > context->dbg_max_dx)
		context->dbg_max_dx = fabs(corr_a);
	if (fabs(corr_b) > context->dbg_max_dy)
		context->dbg_max_dy = fabs(corr_b);
	if (clamped)
		context->dbg_clamped++;
	const double elapsed = context->prev_pose_time - context->dbg_window_start;
	if (elapsed >= 2.0) {
		if (context->stab_debug)
			info("stab: compositor=%.1f fps, tau=%.0f ms, max corr=(%.2f, %.2f) %s, clamped %u frames",
			     (double)(cur->m_nFrameIndex - context->dbg_window_frame_index) / elapsed,
			     context->stab_tau * 1000.0, context->dbg_max_dx, context->dbg_max_dy, unit,
			     context->dbg_clamped);
		if (context->stab_full_lock && context->dbg_cal_n > 2) {
			const int m = context->dbg_cal_n - 1;
			const double mean = context->dbg_cal_sum / m;
			const double var = context->dbg_cal_sum2 / m - mean * mean;
			info("stab: calibration score %.3f (lower = stiller; sweep Mirror Lag to minimize)",
			     sqrt(var > 0.0 ? var : 0.0));
		}
		context->dbg_cal_n = 0;
		context->dbg_cal_sum = context->dbg_cal_sum2 = 0.0;
		context->dbg_window_start = context->prev_pose_time;
		context->dbg_window_frame_index = cur->m_nFrameIndex;
		context->dbg_max_dx = context->dbg_max_dy = 0.0;
		context->dbg_clamped = 0;
	}
}

// Tier 1 fallback: shift the crop rectangle so it follows the smoothed
// orientation. Yaw/pitch only, whole pixels — used when the reprojection
// shader is unavailable.
static void stab_compute_crop(win_openvr *context, const vr::Compositor_FrameTiming *cur,
			      unsigned int *out_x, unsigned int *out_y)
{
	const long max_x = (long)context->device_width - (long)context->width;
	const long max_y = (long)context->device_height - (long)context->height;
	if ((max_x <= 0 && max_y <= 0) ||
	    fabsf(context->proj_right - context->proj_left) < 0.1f ||
	    fabsf(context->proj_bottom - context->proj_top) < 0.1f)
		return; // no headroom (Zoom = 1) or no projection info

	quatd q_a;
	if (!stab_update_filter(context, cur, stab_resolve_pose_delay(context), &q_a))
		return;

	// Where the smoothed forward axis lands in the actual camera's image plane
	const quatd q_err = quat_mul(quat_conj(q_a), context->q_smooth);
	const vec3d f = quat_rotate(q_err, vec3d{0.0, 0.0, -1.0});
	if (!(f.z < -0.5)) { // diverged > 60 deg (or NaN pose), snap back
		context->q_smooth = q_a;
		return;
	}
	const double tx = f.x / -f.z;
	const double ty = f.y / -f.z;

	const double fx = (double)context->device_width / ((double)context->proj_right - (double)context->proj_left);
	const double fy = (double)context->device_height / ((double)context->proj_bottom - (double)context->proj_top);

	const double dx = fx * tx;
	const double dy = -fy * ty;

	if (!std::isfinite(dx) || !std::isfinite(dy)) {
		context->stab_has_state = false; // bad pose data, reseed next frame
		return;
	}

	const long nx = std::lround((double)context->x + dx);
	const long ny = std::lround((double)context->y + dy);
	const long cx = nx < 0 ? 0 : (nx > max_x ? max_x : nx);
	const long cy = ny < 0 ? 0 : (ny > max_y ? max_y : ny);

	// Per-axis anti-windup: when the crop pins at a margin, rebuild the
	// smoothed pose so its correction sits exactly on what was reachable.
	// A dead axis (zero margin) then simply follows the real pose while
	// the other axis keeps its full smoothing.
	const bool clamped = (cx != nx) || (cy != ny);
	if (clamped) {
		const double tx_eff = (double)(cx - (long)context->x) / fx;
		const double ty_eff = (double)(cy - (long)context->y) / -fy;
		vec3d v = {tx_eff, ty_eff, -1.0};
		const double vn = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		v = {v.x / vn, v.y / vn, v.z / vn};
		// shortest-arc rotation taking (0,0,-1) onto v
		const quatd q_align = quat_normalize({1.0 - v.z, v.y, -v.x, 0.0});
		context->q_smooth = quat_normalize(quat_mul(q_a, q_align));
	}

	*out_x = (unsigned int)cx;
	*out_y = (unsigned int)cy;

	stab_debug_stats(context, cur, dx, dy, clamped, "px");
}

// Wide mode: a crop corner is feasible when EITHER eye's frustum covers it
// (the blend weights normalize, so single-eye coverage renders fine). Rays
// live in head space on the union canvas.
static bool stab_corners_ok_wide(win_openvr *context, const quatd &q_err_head, const vec3d &posc_head)
{
	const quatd qA = quat_normalize(quat_mul(quat_conj(context->q_e2h), q_err_head));
	const quatd qB = quat_normalize(quat_mul(quat_conj(context->q_e2h2), q_err_head));
	const vec3d pA = quat_rotate(quat_conj(context->q_e2h), posc_head);
	const vec3d pB = quat_rotate(quat_conj(context->q_e2h2), posc_head);
	const double W = (double)context->device_width, H = (double)context->device_height;
	const double lA = (double)context->proj_left, rA = (double)context->proj_right;
	const double tA = (double)context->proj_top, bA = (double)context->proj_bottom;
	const double lB = (double)context->proj2_left, rB = (double)context->proj2_right;
	const double tB = (double)context->proj2_top, bB = (double)context->proj2_bottom;
	const double iuA = 2.0 * (rA - lA) / W, ivA = 2.0 * (bA - tA) / H;
	const double iuB = 2.0 * (rB - lB) / W, ivB = 2.0 * (bB - tB) / H;
	const double lC = (double)context->projc_left, rC = (double)context->projc_right;
	const double tC = (double)context->projc_top, bC = (double)context->projc_bottom;
	const double CW = (double)context->canvas_w, CH = (double)context->canvas_h;
	const double u0 = (double)context->x / CW, u1 = (double)(context->x + context->width) / CW;
	const double v0 = (double)context->y / CH, v1 = (double)(context->y + context->height) / CH;
	// Sample the full crop PERIMETER (4 points per edge), not just corners:
	// the either-eye coverage union is non-convex on canted displays (tilted
	// per-eye quads meet in a "tent"), so all four corners can be covered by
	// different eyes while an edge midspan escapes coverage entirely.
	for (int s = 0; s < 16; s++) {
		const int edge = s >> 2;
		const double f = (double)(s & 3) / 4.0;
		double u, v;
		switch (edge) {
		case 0:
			u = u0 + f * (u1 - u0), v = v0; // top
			break;
		case 1:
			u = u1, v = v0 + f * (v1 - v0); // right
			break;
		case 2:
			u = u1 - f * (u1 - u0), v = v1; // bottom
			break;
		default:
			u = u0, v = v1 - f * (v1 - v0); // left
			break;
		}
		const double tx = lC + u * (rC - lC);
		const double ty = bC - v * (bC - tC);
		const vec3d ray = {tx, ty, -1.0};
		bool ok = false;
		const vec3d dra = quat_rotate(qA, ray);
		const vec3d da = {dra.x - pA.x, dra.y - pA.y, dra.z - pA.z};
		if (da.z < -0.1) {
			const double px = da.x / -da.z, py = da.y / -da.z;
			ok = px >= lA + iuA && px <= rA - iuA && py >= tA + ivA && py <= bA - ivA;
		}
		if (!ok) {
			const vec3d drb = quat_rotate(qB, ray);
			const vec3d db = {drb.x - pB.x, drb.y - pB.y, drb.z - pB.z};
			if (db.z < -0.1) {
				const double px = db.x / -db.z, py = db.y / -db.z;
				ok = px >= lB + iuB && px <= rB - iuB && py >= tB + ivB && py <= bB - ivB;
			}
		}
		if (!ok)
			return false;
	}
	return true;
}

// Mode dispatch: q_err in head space, posc in head space.
static bool stab_corners_feasible(win_openvr *context, const quatd &q_err_head, const vec3d &posc_head)
{
	if (context->wide_active)
		return stab_corners_ok_wide(context, q_err_head, posc_head);
	return stab_corners_ok(context, q_err_head, quat_rotate(quat_conj(context->q_e2h), posc_head));
}

// True when every corner of the output crop, corrected by q_err_head and the
// positional tangent offset, still samples inside the mirror frustum (with a
// ~2 texel safety inset).
static bool stab_corners_ok(win_openvr *context, const quatd &q_err_head, const vec3d &posc_eye)
{
	const quatd q_e = quat_mul(quat_mul(quat_conj(context->q_e2h), q_err_head), context->q_e2h);
	const double W = (double)context->device_width, H = (double)context->device_height;
	const double l = (double)context->proj_left, r = (double)context->proj_right;
	const double t = (double)context->proj_top, b = (double)context->proj_bottom;
	const double iu = 2.0 * (r - l) / W;
	const double iv = 2.0 * (b - t) / H;
	const double us[2] = {(double)context->x / W, (double)(context->x + context->width) / W};
	const double vs[2] = {(double)context->y / H, (double)(context->y + context->height) / H};
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			const double tx = l + us[i] * (r - l);
			const double ty = b - vs[j] * (b - t);
			const vec3d dr = quat_rotate(q_e, vec3d{tx, ty, -1.0});
			const vec3d d = {dr.x - posc_eye.x, dr.y - posc_eye.y, dr.z - posc_eye.z};
			if (!(d.z < -0.1))
				return false;
			const double px = d.x / -d.z, py = d.y / -d.z;
			if (!(px >= l + iu && px <= r - iu && py >= t + iv && py <= b - iv))
				return false;
		}
	}
	return true;
}

// Ground-truth recorder: one CSV row per compositor frame with the raw and
// smoothed pose so filter behavior can be analyzed offline against real
// motion data instead of guesses. Written to the OBS log folder.
static void stab_telemetry_write(win_openvr *context, const vr::Compositor_FrameTiming *cur, int pose_delay,
				 bool frozen, bool clamped, double corr_deg, const vec3d &posc_eye,
				 const StabPairInfo &pr)
{
	if (!context->stab_telemetry) {
		if (context->stab_telemetry_file) {
			fclose(context->stab_telemetry_file);
			context->stab_telemetry_file = nullptr;
		}
		return;
	}
	if (!context->stab_telemetry_file) {
		const char *appdata = getenv("APPDATA");
		if (!appdata)
			return;
		char path[512];
		snprintf(path, sizeof(path), "%s\\obs-studio\\logs\\stab-telemetry.csv", appdata);
		context->stab_telemetry_file = fopen(path, "w");
		if (!context->stab_telemetry_file) {
			warn("stab: could not open telemetry file %s", path);
			context->stab_telemetry = false;
			return;
		}
		info("stab: recording telemetry to %s", path);
		fprintf(context->stab_telemetry_file,
			"time,frame,delay,dframe,phase_ms,extrap_ms,eff_lag_ms,lag_mode,cap_used,cap_phase_ms,"
			"pair_future,vsync_ok,vsync_ms,vsync_ctr,req_delay,eff_delay,age_ms,"
			"presents,dropped,mispres,flags,"
			"predicted,vs_ready,vs_view,crender_start_ms,crender_gpu_ms,crender_cpu_ms,"
			"newframe_ready_ms,transfer_ms,frozen,clamped,qa_w,qa_x,qa_y,qa_z,qs_w,qs_x,qs_y,qs_z,"
			"pa_x,pa_y,pa_z,ps_x,ps_y,ps_z,corr_deg,posc_x,posc_y,posc_z\n");
		context->stab_telemetry_lines = 0;
	}
	const quatd &qa = context->stab_q_ref;
	const quatd &qs = context->q_smooth;
	fprintf(context->stab_telemetry_file,
		"%.6f,%u,%d,%u,%.4f,%.4f,%.4f,%d,%d,%.4f,%d,%d,%.4f,%llu,%d,%d,%.4f,%u,%u,%u,0x%X,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,"
		"%d,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
		"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.6f,%.6f,%.6f\n",
		context->prev_pose_time, cur->m_nFrameIndex, pose_delay, pr.dframe, pr.phase_ms, pr.extrap_ms,
		pr.eff_lag_ms, pr.lag_mode, pr.cap_used, pr.cap_phase_ms, pr.pair_future, pr.vsync_ok,
		pr.vsync_phase_ms, (unsigned long long)pr.vsync_counter,
		pr.req_delay, pr.eff_delay, pr.age_ms,
		pr.presents, pr.dropped, pr.mispres, pr.flags, pr.predicted, pr.vs_ready, pr.vs_view,
		pr.crender_start_ms, pr.crender_gpu_ms, pr.crender_cpu_ms, pr.newframe_ready_ms, pr.transfer_ms,
		frozen ? 1 : 0, clamped ? 1 : 0, qa.w, qa.x,
		qa.y, qa.z, qs.w, qs.x, qs.y, qs.z, context->stab_p_raw.x, context->stab_p_raw.y, context->stab_p_raw.z,
		context->p_smooth.x, context->p_smooth.y, context->p_smooth.z, corr_deg, posc_eye.x, posc_eye.y,
		posc_eye.z);
	if (++context->stab_telemetry_lines % 90 == 0)
		fflush(context->stab_telemetry_file);
}

// Milestone B: render the stabilized view by reprojecting the mirror texture
// into texCrop on the plugin's own device. Replaces the crop copy entirely.
// Returns false if the pass could not run (caller falls back to the copy).
static bool stab_render_warp(win_openvr *context, const vr::Compositor_FrameTiming *cur)
{
	if (!context->stab_shader_ok || !context->mirrorSrv || !context->stabRTV)
		return false;
	if (context->wide_active && !context->mirrorSrv2)
		return false;
	if (fabsf(context->proj_right - context->proj_left) < 0.1f ||
	    fabsf(context->proj_bottom - context->proj_top) < 0.1f)
		return false;

	quatd q_err = {1.0, 0.0, 0.0, 0.0};
	vec3d posc_h = {0.0, 0.0, 0.0}; // head frame
	bool clamped = false;
	bool frozen = false;
	StabPairInfo pair = {}; // req_delay -1 until the pairing runs
	pair.req_delay = -1;
	// Boundary-locked capture first: warp the newest complete poller slot -
	// pixels captured at a fixed mid-slot phase regardless of canvas rate.
	// Falls back to the render-tick hold when the poller is degraded or the
	// slots are not ready, then to the live mirror.
	int cap_slot = -1;
	if (context->stab_hold && !context->poll_degraded && context->stab_poll_thread) {
		for (int i = 0; i < 2; i++) {
			const int s = (context->cap_newest + 2 - i) & 1;
			if (InterlockedCompareExchange(&context->cap_valid[s], 1, 1) && context->cap_srv[s][0] &&
			    (!context->wide_active || context->cap_srv[s][1])) {
				cap_slot = s;
				break;
			}
		}
	}
	const bool use_cap = cap_slot >= 0;
	const bool use_hold = !use_cap && context->stab_hold && context->hold_valid && context->holdSrv &&
			      (!context->wide_active || context->holdSrv2);
	ID3D11ShaderResourceView *srcA = use_cap    ? context->cap_srv[cap_slot][0]
					 : use_hold ? context->holdSrv.Get()
						    : context->mirrorSrv.Get();
	ID3D11ShaderResourceView *srcB =
		context->wide_active ? (use_cap    ? context->cap_srv[cap_slot][1]
					: use_hold ? context->holdSrv2.Get()
						   : context->mirrorSrv2.Get())
				     : nullptr;
	quatd q_a_eff = {1.0, 0.0, 0.0, 0.0}; // pose the displayed frame was rendered with
	bool corrected = false;               // fresh q_err/posc this frame (needs clamp)

	if (context->stabilize) {
		// Warp the LIVE mirror against its frame-based matched pose: the pose
		// GetFrameTiming(stab_pair_frames) - the number of compositor frames
		// the mirror lags the in-flight frame. Frame-based (not wall-clock), so
		// pairing stays consistent when the frametime jitters under GPU load,
		// and each frame (including motion-smoothed ones) is paired with the
		// pose it was rendered with - the smoothing filter handles them.
		quatd q_a;
		vec3d p_a;
		double t_a = 0.0;
		stab_track_cadence(context, cur);
		const double eff_lag = stab_effective_lag_s(context);
		int want;
		if (use_cap) {
			// Pairing bound by the slot's own capture stamp. On a failed
			// ring lookup, REUSE the last pairing - never switch to a
			// different selector mid-session: a consistent repeat glides
			// through the reanchor, an alternating rule-change is shake.
			const int64_t target = context->cap_qpc[cap_slot] +
					       (int64_t)(eff_lag * (double)context->stab_qpc_freq);
			uint32_t idx = stab_record_at_qpc(context, target);
			// Future-target guard (panel A0, HIGH): with positive lag the
			// target can sit past the newest ring stamp while its deciding
			// boundary has ALREADY happened - the render gate saw the new
			// index up to ~1 ms before the poller stamped it. The ring then
			// returns the previous record: found-but-WRONG, alternating with
			// tick phase. In that window the render thread's own
			// GetFrameTiming(0) is fresher than the ring: the newest record
			// IS cur.
			{
				const int rpos = (int)context->stab_poll_pos;
				if (target > context->stab_poll_qpc[rpos] &&
				    (int32_t)(cur->m_nFrameIndex - context->stab_poll_idx[rpos]) > 0) {
					idx = cur->m_nFrameIndex;
					pair.pair_future = 1;
				}
			}
			want = idx ? (int)(cur->m_nFrameIndex - idx)
				   : (context->stab_has_sel ? (int)(cur->m_nFrameIndex - context->stab_last_sel)
							    : context->stab_pair_frames);
			want = std::min(std::max(want, 0), 4);
			context->stab_extrap_s = 0.0; // never predict from a capture
		} else if (use_hold) {
			// Retrospective pairing: which record was newest when the GPU
			// actually sampled the pixels we are about to warp? That instant
			// is hold_qpc + the effective lag, and it is now in the past, so
			// the poller's stamps resolve it exactly. Negative lag reaches
			// further back - the reprojected pipeline holds the mirror one
			// record behind its timestamp.
			const int64_t target =
				context->hold_qpc + (int64_t)(eff_lag * (double)context->stab_qpc_freq);
			const uint32_t idx = stab_record_at_qpc(context, target);
			want = idx ? (int)(cur->m_nFrameIndex - idx) : context->stab_pair_frames;
			want = std::min(std::max(want, 0), 4);
			context->stab_extrap_s = 0.0; // never predict while holding
		} else {
			want = stab_resolve_pair_frames(context, cur, eff_lag);
		}

		// Re-anchor guard. If the selected record does not advance STRICTLY,
		// pose_time repeats, dt is exactly 0, and stab_filter_core's dt <= 0
		// branch returns false - leaving q_err at identity so the frame renders
		// completely UNWARPED. Under Full Lock that is the whole accumulated
		// correction appearing as one jump, far worse than a one-record residual.
		// The re-anchor path absorbs the timestamp jump and still applies the
		// correction, so the offset change glides.
		const uint32_t sel = cur->m_nFrameIndex - (uint32_t)want;
		if (context->stab_has_sel && (int32_t)(sel - context->stab_last_sel) <= 0)
			context->stab_time_reanchor = true;
		context->stab_last_sel = sel;
		context->stab_has_sel = true;

		bool have = stab_fetch_pose(cur, want, &q_a, &p_a, &t_a, &frozen, &pair);

		// Compositor drops emit adjacent records with equal or slightly
		// REGRESSED timestamps (measured: 0 to -45 us, always dropped=1). The
		// index guard above cannot see that - the index advances - but dt <= 0
		// would hit stab_filter_core's resync branch, which renders the frame
		// completely UNWARPED: the whole accumulated correction appears as a
		// single visible pop. Route it through the reanchor instead: the
		// correction still applies, the clock re-bases, the frame glides. The
		// resync branch remains for genuine hitches (dt > 0.25 s).
		if (have && context->stab_has_state && !context->stab_time_reanchor &&
		    t_a <= context->prev_pose_time && context->prev_pose_time - t_a < 0.05)
			context->stab_time_reanchor = true;

		// Extrapolate the paired pose forward to where the head will be when the
		// GPU actually samples the mirror. The step is measured from the two
		// newest records, so it needs no assumed rate; capped so a tracking
		// glitch cannot throw the correction.
		if (have && context->stab_extrap_s > 0.0) {
			vr::Compositor_FrameTiming a = {}, b = {};
			a.m_nSize = b.m_nSize = sizeof(vr::Compositor_FrameTiming);
			if (vr::VRCompositor()->GetFrameTiming(&a, (uint32_t)want) &&
			    vr::VRCompositor()->GetFrameTiming(&b, (uint32_t)want + 1) &&
			    a.m_HmdPose.bPoseIsValid && b.m_HmdPose.bPoseIsValid) {
				const double span = a.m_flSystemTimeInSeconds - b.m_flSystemTimeInSeconds;
				if (span > 0.001) {
					const quatd qb = quat_normalize(quat_from_hmd34(b.m_HmdPose.mDeviceToAbsoluteTracking));
					const quatd step = quat_mul(quat_conj(qb), q_a); // rotation over span
					double f = context->stab_extrap_s / span;
					if (f > 2.0)
						f = 2.0; // never extrapolate more than two frames
					q_a = quat_normalize(quat_mul(q_a, quat_slerp(quatd{1.0, 0.0, 0.0, 0.0}, step, f)));
				}
			}
		}
		pair.dframe = cur->m_nFrameIndex - context->lastFrame;
		pair.extrap_ms = context->stab_extrap_s * 1000.0;
		pair.eff_lag_ms = eff_lag * 1000.0;
		pair.lag_mode = context->stab_lag_n; // vsyncs/record held (1 = native)
		pair.cap_used = use_cap ? 1 : 0;
		pair.cap_phase_ms = use_cap ? context->cap_phase_ms[cap_slot] : -1.0;
		float since_vsync = 0.0f;
		uint64_t vsync_frame = 0;
		pair.vsync_ok = (vr::VRSystem() &&
				 vr::VRSystem()->GetTimeSinceLastVsync(&since_vsync, &vsync_frame))
					? 1
					: 0;
		pair.vsync_phase_ms = since_vsync * 1000.0;
		pair.vsync_counter = vsync_frame;
		const double ph = stab_phase_of(context, cur->m_nFrameIndex);
		pair.phase_ms = ph >= 0.0 ? ph * 1000.0 : -1.0;
		if (!have)
			context->stab_has_state = false;
		else if (stab_filter_core(context, cur, q_a, p_a, t_a)) {
			q_a_eff = q_a;
			q_err = quat_mul(quat_conj(q_a), context->q_smooth);
			posc_h = context->stab_posc;
			corrected = true;
		}
	}

	// Corner-feasibility clamp (anti-windup), shared by both paths. q_a_eff is
	// the pose the displayed frame was rendered with; the rotational excess
	// folds back into q_smooth so the correction rides the largest feasible
	// fraction (the positional state is self-bounded at 2 cm).
	if (corrected && !context->stab_black_edges && !stab_corners_feasible(context, q_err, posc_h)) {
		const quatd q_target = context->q_smooth;
		const vec3d posc_full = posc_h;
		double lo = 0.0, hi = 1.0;
		for (int it = 0; it < 12; it++) {
			const double mid = 0.5 * (lo + hi);
			const quatd q_try = quat_mul(quat_conj(q_a_eff), quat_slerp(q_a_eff, q_target, mid));
			const vec3d p_try = {posc_full.x * mid, posc_full.y * mid, posc_full.z * mid};
			if (stab_corners_feasible(context, q_try, p_try))
				lo = mid;
			else
				hi = mid;
		}
		context->q_smooth = quat_normalize(quat_slerp(q_a_eff, q_target, lo));
		q_err = quat_mul(quat_conj(q_a_eff), context->q_smooth);
		posc_h = {posc_full.x * lo, posc_full.y * lo, posc_full.z * lo};
		clamped = true;
	}
	if (context->stabilize) {
		context->q_err_last = q_err;
		context->stab_posc_last = posc_h;
	}

	// Per-eye constants. Single mode: canvas rays live in the eye's own
	// space, so the correction is conjugated into it (conj(e)*q*e). Wide
	// mode: canvas rays live in HEAD space, so each eye's matrix carries
	// the head->eye rotation itself (conj(e)*q).
	const quatd qA = context->wide_active
				 ? quat_mul(quat_conj(context->q_e2h), q_err)
				 : quat_mul(quat_mul(quat_conj(context->q_e2h), q_err), context->q_e2h);
	const vec3d pA = quat_rotate(quat_conj(context->q_e2h), posc_h);

	StabCBData cb = {};
	cb.frC[0] = context->projc_left;
	cb.frC[1] = context->projc_right;
	cb.frC[2] = context->projc_top;
	cb.frC[3] = context->projc_bottom;
	cb.frA[0] = context->proj_left;
	cb.frA[1] = context->proj_right;
	cb.frA[2] = context->proj_top;
	cb.frA[3] = context->proj_bottom;
	cb.crop[0] = (float)context->x / (float)context->canvas_w;
	cb.crop[1] = (float)context->y / (float)context->canvas_h;
	cb.crop[2] = (float)context->width / (float)context->canvas_w;
	cb.crop[3] = (float)context->height / (float)context->canvas_h;
	cb.misc[0] = context->stab_encode_srgb ? 1.0f : 0.0f;
	cb.misc[1] = context->wide_active ? 1.0f : 0.0f;
	cb.misc[2] = STAB_BLEND_BAND;
	cb.misc[3] = context->righteye ? 1.0f : -1.0f; // dominant right eye: nasal = its left edge
	cb.poscA[0] = (float)pA.x;
	cb.poscA[1] = (float)pA.y;
	cb.poscA[2] = (float)pA.z;
	quat_to_mat4(quat_normalize(qA), cb.RdA);
	if (context->wide_active) {
		const quatd qB = quat_mul(quat_conj(context->q_e2h2), q_err);
		const vec3d pB = quat_rotate(quat_conj(context->q_e2h2), posc_h);
		cb.frB[0] = context->proj2_left;
		cb.frB[1] = context->proj2_right;
		cb.frB[2] = context->proj2_top;
		cb.frB[3] = context->proj2_bottom;
		cb.poscB[0] = (float)pB.x;
		cb.poscB[1] = (float)pB.y;
		cb.poscB[2] = (float)pB.z;
		quat_to_mat4(quat_normalize(qB), cb.RdB);
	}

	ID3D11DeviceContext *ctx = context->shared_context.Get();
	ctx->UpdateSubresource(context->stabCB.Get(), 0, nullptr, &cb, 0, 0);
	ID3D11RenderTargetView *rtv = context->stabRTV.Get();
	ctx->OMSetRenderTargets(1, &rtv, nullptr);
	D3D11_VIEWPORT vp = {0.0f, 0.0f, (float)context->width, (float)context->height, 0.0f, 1.0f};
	ctx->RSSetViewports(1, &vp);
	ctx->IASetInputLayout(nullptr);
	ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	ctx->VSSetShader(context->stabVS.Get(), nullptr, 0);
	ctx->PSSetShader(context->stabPS.Get(), nullptr, 0);
	ID3D11Buffer *cbuf = context->stabCB.Get();
	ctx->PSSetConstantBuffers(0, 1, &cbuf);
	ID3D11ShaderResourceView *srvs[2] = {srcA, context->wide_active ? srcB : srcA};
	ctx->PSSetShaderResources(0, 2, srvs);
	ID3D11SamplerState *smp = context->stabSampler.Get();
	ctx->PSSetSamplers(0, 1, &smp);
	ctx->RSSetState(context->stabRaster.Get());
	ctx->OMSetBlendState(nullptr, nullptr, 0xffffffff);
	ctx->OMSetDepthStencilState(nullptr, 0);
	ctx->Draw(3, 0);
	ID3D11ShaderResourceView *nullsrvs[2] = {nullptr, nullptr};
	ctx->PSSetShaderResources(0, 2, nullsrvs);
	// Legacy render-tick capture: only when the poller cannot provide slots
	// (thread failed or cadence degraded). With the boundary-locked capture
	// healthy, copying here too would just double the bandwidth.
	if (context->stab_hold && (context->poll_degraded || !context->stab_poll_thread))
		stab_hold_capture(context, ctx, cur);
	ID3D11RenderTargetView *nullrtv = nullptr;
	ctx->OMSetRenderTargets(1, &nullrtv, nullptr);

	if (context->stabilize) {
		const double corr_deg = 2.0 * acos(std::min(1.0, fabs(q_err.w))) * 57.29577951308232;
		const double pos_mm = sqrt(posc_h.x * posc_h.x + posc_h.y * posc_h.y + posc_h.z * posc_h.z) *
				      context->stab_pos_depth * 1000.0;
		stab_debug_stats(context, cur, corr_deg, pos_mm, clamped, "deg/mm");
		stab_telemetry_write(context, cur, context->stab_pair_frames, frozen, clamped, corr_deg, pA, pair);
	}
	return true;
}

static void win_openvr_render(void *data, gs_effect_t *effect)
{
	win_openvr *context = (win_openvr *)data;

	if (!context->active) {
		return;
	}

	if (!context->initialized) {
		// Active & want to render but not initialized - attempt to init
		win_openvr_init1(data);
	}

	if (vr::VRCompositor()) {
		vr::Compositor_FrameTiming frameTiming = {};
		frameTiming.m_nSize = sizeof(vr::Compositor_FrameTiming);
		if (vr::VRCompositor()->GetFrameTiming(&frameTiming, 0)) {
			if (frameTiming.m_nFrameIndex != context->lastFrame) {
				// One lock over the whole D3D body: pointer checks and
				// use become atomic against the poller's capture and
				// against init/update mutating the sources.
				StabGpuGuard gpu(&context->stab_gpu_lock);
				if (context->texCrop && context->tex) {
					bool warped = false;
					if ((context->stabilize || context->wide_active) && context->stab_shader_ok)
						warped = stab_render_warp(context, &frameTiming);
					// In wide mode the crop rect lives on the union canvas and
					// cannot be copied out of a single mirror - skip rather
					// than issue an out-of-bounds CopySubresourceRegion.
					if (!warped && !context->wide_active) {
						unsigned int crop_x = context->x;
						unsigned int crop_y = context->y;
						if (context->stabilize)
							stab_compute_crop(context, &frameTiming, &crop_x, &crop_y);
						D3D11_BOX poksi = {crop_x, crop_y, 0, crop_x + context->width, crop_y + context->height, 1};
						context->shared_context->CopySubresourceRegion(context->texCrop.Get(), 0, 0, 0, 0, context->tex.Get(), 0, &poksi);
					}
					context->shared_context->Flush();
					context->lastFrame = frameTiming.m_nFrameIndex;
				}
			}
		}
	}

	effect = obs_get_base_effect(OBS_EFFECT_OPAQUE);

	if (context->texture) {
		while (gs_effect_loop(effect, "Draw")) {
			obs_source_draw(context->texture, 0, 0, 0, 0, false);
		}
	}
}

static void win_openvr_tick(void *data, float seconds)
{
	UNUSED_PARAMETER(seconds);

	struct win_openvr *context = (win_openvr *)data;

	context->active = obs_source_showing(context->source);

	vr::VREvent_t e;
	if (vr::VRSystem() != NULL) {
		if (vr::VRSystem()->PollNextEvent(&e, sizeof(vr::VREvent_t))) {
			if (e.eventType == vr::VREvent_Quit) {
				// Without this SteamVR will kill OBS process when it exits
				win_openvr_deinit(data);
			}
		}
	}

	if (!context->initialized && context->active) {
		win_openvr_init1(data);
	}
}

static bool set_vis(obs_properties_t *props, const char *name, bool vis)
{
	obs_property_t *p = obs_properties_get(props, name);
	if (!p || obs_property_visible(p) == vis)
		return false;
	obs_property_set_visible(p, vis);
	return true;
}

// Returning true rebuilds the whole properties panel, which kills an active
// slider drag - so only return true when visibility actually changed.
static bool stab_ui_modd(obs_properties_t *props, obs_property_t *property, obs_data_t *settings)
{
	bool on = obs_data_get_bool(settings, "stabilize");
	bool custom = obs_data_get_int(settings, "stab_preset") == 0;
	bool nomargin = obs_data_get_double(settings, "scale_factor") < 1.01;
	bool pos = obs_data_get_bool(settings, "stab_pos_comp");
	// Collapsed panel (UI v1): main = Strength + Level Horizon + the zoom
	// warning; everything else behind Show Advanced. The investigation-era
	// pairing controls have no UI at all - the machinery is always on, and
	// their settings keys stay readable for scripts.
	const bool adv = obs_data_get_bool(settings, "stab_advanced");
	bool changed = set_vis(props, "stab_zoom_warning", on && nomargin);
	changed |= set_vis(props, "stab_preset", on);
	changed |= set_vis(props, "stab_roll_lock", on);
	changed |= set_vis(props, "stab_advanced", on);
	changed |= set_vis(props, "stab_filter", on && adv);
	changed |= set_vis(props, "stab_smooth_ms", on && custom);
	changed |= set_vis(props, "stab_pos_comp", on && adv);
	changed |= set_vis(props, "stab_pos_depth", on && adv && pos);
	changed |= set_vis(props, "stab_lag_base_ms", on && adv);
	changed |= set_vis(props, "stab_full_lock", on && adv);
	changed |= set_vis(props, "stab_debug", on && adv);
	changed |= set_vis(props, "stab_telemetry", on && adv);

	// One Euro only smooths at this rate while the head is at rest - rename
	// the slider so the number isn't read as a fixed delay in that mode.
	obs_property_t *sm = obs_properties_get(props, "stab_smooth_ms");
	if (sm) {
		const char *want = obs_data_get_int(settings, "stab_filter") == 1
					   ? obs_module_text("Smoothing Time at Rest (ms)")
					   : obs_module_text("Smoothing Time (ms)");
		const char *cur = obs_property_description(sm);
		if (!cur || strcmp(cur, want) != 0) {
			obs_property_set_description(sm, want);
			changed = true;
		}
	}
	return changed;
}

static bool ar_modd(obs_properties_t *props, obs_property_t *property, obs_data_t *settings)
{
	double aspect_ratio = obs_data_get_double(settings, "aspect_ratio");

	bool custom_active = (aspect_ratio == 0.0);

	obs_property_t *custom_width = obs_properties_get(props, "custom_aspect_width");
	obs_property_t *custom_height = obs_properties_get(props, "custom_aspect_height");

	obs_property_set_visible(custom_width, custom_active);
	obs_property_set_visible(custom_height, custom_active);

	return true;
}

static obs_properties_t *win_openvr_properties(void *data)
{
	win_openvr *context = (win_openvr *)data;

	obs_properties_t *props = obs_properties_create();
	obs_property_t *p;

	p = obs_properties_add_list(props, "eye_mode", obs_module_text("Eye"), OBS_COMBO_TYPE_LIST,
				    OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(p, obs_module_text("Left"), 0);
	obs_property_list_add_int(p, obs_module_text("Right"), 1);
	obs_property_list_add_int(p, obs_module_text("Both - Wide (Left eye dominant)"), 2);
	obs_property_list_add_int(p, obs_module_text("Both - Wide (Right eye dominant)"), 3);
	obs_property_set_long_description(
		p, obs_module_text("Wide composites both eyes into a wider horizontal FOV, aligned at far "
				   "distance. The dominant eye fills the frame wherever it has coverage; the "
				   "other eye fades in near the far edge, so the blend seam sits off-center. "
				   "Nearby objects can ghost at the seam - distant scenery merges cleanly."));

	// Preset aspect ratios
	p = obs_properties_add_list(props, "aspect_ratio", obs_module_text("Aspect Ratio"), OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_FLOAT);
	obs_property_list_add_float(p, "Native", -1.0);
	obs_property_list_add_float(p, "16:9", 16.0 / 9.0);
	obs_property_list_add_float(p, "4:3", 4.0 / 3.0);
	obs_property_list_add_float(p, "Custom", 0.0);

	obs_property_set_modified_callback(p, ar_modd);

	p = obs_properties_add_int(props, "custom_aspect_width", obs_module_text("Ratio Width"), 1, 100, 1);
	obs_property_set_visible(p, false);
	p = obs_properties_add_int(props, "custom_aspect_height", obs_module_text("Ratio Height"), 1, 100, 1);
	obs_property_set_visible(p, false);

	// Pan and zoom
	p = obs_properties_add_float_slider(props, "scale_factor", obs_module_text("Zoom"), 1.0, 5.0, 0.01);
	obs_property_set_modified_callback(p, stab_ui_modd);
	p = obs_properties_add_int(props, "x_offset", obs_module_text("Horizontal Offset"), -10000, 10000, 1);
	p = obs_properties_add_int(props, "y_offset", obs_module_text("Vertical Offset"), -10000, 10000, 1);

	// Stabilization
	p = obs_properties_add_bool(props, "stabilize", obs_module_text("Stabilization (Experimental)"));
	obs_property_set_long_description(
		p, obs_module_text("Smooths head rotation (yaw/pitch/roll) by reprojecting the mirror frame. "
				   "Uses the Zoom margin to hide corrections; at Zoom 1.0 black edges appear."));
	obs_property_set_modified_callback(p, stab_ui_modd);
	p = obs_properties_add_text(props, "stab_zoom_warning",
				    obs_module_text("Zoom is 1.0, so corrections will reveal black edges. "
						    "Increase Zoom (1.2+) to hide them."),
				    OBS_TEXT_INFO);
	obs_property_text_set_info_type(p, OBS_TEXT_INFO_WARNING);
	p = obs_properties_add_list(props, "stab_preset", obs_module_text("Strength"), OBS_COMBO_TYPE_LIST,
				    OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(p, obs_module_text("Low"), 1);
	obs_property_list_add_int(p, obs_module_text("Medium"), 2);
	obs_property_list_add_int(p, obs_module_text("High"), 3);
	obs_property_list_add_int(p, obs_module_text("Custom"), 0);
	obs_property_set_modified_callback(p, stab_ui_modd);
	// Placed directly under Strength: choosing Custom reveals it in place, no
	// trip through Advanced (the choice itself was the opt-in).
	p = obs_properties_add_int_slider(props, "stab_smooth_ms", obs_module_text("Smoothing Time (ms)"), 50, 2000, 10);
	obs_property_set_long_description(
		p, obs_module_text("Time constant of the head-direction average the view is reprojected to. "
				   "Higher = steadier output that trails further behind deliberate turns. "
				   "Presets: Low 200, Medium 500, High 1000."));
	p = obs_properties_add_list(props, "stab_filter", obs_module_text("Smoothing Filter"), OBS_COMBO_TYPE_LIST,
				    OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(p, obs_module_text("Damped Average (constant smoothing)"), 0);
	obs_property_list_add_int(p, obs_module_text("One Euro (adaptive)"), 1);
	obs_property_set_modified_callback(p, stab_ui_modd);
	obs_property_set_long_description(
		p, obs_module_text("Both use the Smoothing Time as the at-rest time constant. Damped Average "
				   "suppresses shake equally at rest and in motion but trails deliberate turns. "
				   "One Euro speeds up with head motion: far less trail, but fast movement also "
				   "lets more shake through."));
	p = obs_properties_add_bool(props, "stab_roll_lock", obs_module_text("Roll Lock"));
	obs_property_set_long_description(
		p, obs_module_text("Keeps the horizon gravity-level instead of following smoothed head roll - "
				   "the viewer never sees the camera tilt."));
	p = obs_properties_add_bool(props, "stab_advanced", obs_module_text("Show Advanced Settings"));
	obs_property_set_long_description(
		p, obs_module_text("Filter choice, positional compensation, the Mirror Lag calibration, and "
				   "diagnostics. The defaults are right for almost everyone."));
	obs_property_set_modified_callback(p, stab_ui_modd);
	p = obs_properties_add_bool(props, "stab_full_lock", obs_module_text("Calibration: Freeze View"));
	obs_property_set_long_description(
		p, obs_module_text("Freezes the view to the heading captured when you enable it - for "
				   "calibrating Mirror Lag. Shake the headset and sweep Mirror Lag until the "
				   "image is dead still; a calibration score prints to the OBS log every 2 s "
				   "(lower is better). Turn OFF for normal use - it fights deliberate turning."));
	p = obs_properties_add_bool(props, "stab_pos_comp", obs_module_text("Positional Jitter Compensation (Experimental)"));
	obs_property_set_long_description(
		p, obs_module_text("Counteracts small positional shake (tremor, headset wobble) at an assumed "
				   "scene depth. Capped at 2 cm so deliberate movement is unaffected."));
	obs_property_set_modified_callback(p, stab_ui_modd);
	p = obs_properties_add_float_slider(props, "stab_pos_depth", obs_module_text("Reference Depth (m)"), 0.5, 100.0, 0.5);
	obs_property_set_long_description(
		p, obs_module_text("Scene distance that gets steadied perfectly; content at other depths keeps "
				   "some parallax shake. 1.0-2.0 m suits close-range games; push toward 100 m "
				   "(effectively infinite) for distant scenery / horizons - the positional "
				   "correction shrinks with depth, so far content stops being pushed around."));
	p = obs_properties_add_float_slider(props, "stab_lag_base_ms", obs_module_text("Mirror Lag (ms)"), -20.0, 40.0,
					   0.5);
	obs_property_set_long_description(
		p, obs_module_text("The one calibration: how long after a frame is composited its pixels reach "
				   "the mirror on THIS rig (+8.5 here). Auto-adjusted per load and refresh - "
				   "reprojected games derive their value from this one. To calibrate: check "
				   "Calibration: Freeze View, shake the headset, sweep until dead still."));
	p = obs_properties_add_bool(props, "stab_debug", obs_module_text("Stabilization: Debug Logging"));
	p = obs_properties_add_bool(props, "stab_telemetry", obs_module_text("Stabilization: Record Telemetry (CSV)"));
	obs_property_set_long_description(
		p, obs_module_text("Writes per-frame pose data to stab-telemetry.csv in the OBS log folder "
				   "for offline analysis. Leave off unless diagnosing."));

	// data is NULL when libobs asks for type-level properties (scripts,
	// obs-websocket) with no live instance to read visibility state from.
	if (context) {
		obs_data_t *settings = obs_source_get_settings(context->source);
		ar_modd(props, NULL, settings);
		stab_ui_modd(props, NULL, settings);
		obs_data_release(settings);
	}

	return props;
}

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("win-openvr", "en-US")

bool obs_module_load(void)
{
	obs_source_info info = {};
	info.id = "openvr_capture";
	info.type = OBS_SOURCE_TYPE_INPUT;
	info.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_CUSTOM_DRAW;
	info.get_name = win_openvr_get_name;
	info.create = win_openvr_create;
	info.destroy = win_openvr_destroy;
	info.update = win_openvr_update;
	info.get_defaults = win_openvr_defaults;
	info.show = win_openvr_show;
	info.hide = win_openvr_hide;
	info.get_width = win_openvr_getwidth;
	info.get_height = win_openvr_getheight;
	info.video_render = win_openvr_render;
	info.video_tick = win_openvr_tick;
	info.get_properties = win_openvr_properties;
	obs_register_source(&info);
	return true;
}
