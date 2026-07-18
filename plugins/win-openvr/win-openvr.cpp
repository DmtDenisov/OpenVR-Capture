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
	bool stab_pos_comp;
	double stab_pos_depth;
	int stab_pose_delay;
	bool stab_debug;
	bool stab_telemetry;
	FILE *stab_telemetry_file;
	uint32_t stab_telemetry_lines;

	quatd stab_q_ref; // raw pose of the displayed frame (telemetry)

	// Per-eye projection frustum tangents from GetProjectionRaw
	float proj_left, proj_right, proj_top, proj_bottom;     // dominant eye
	float proj2_left, proj2_right, proj2_top, proj2_bottom; // other eye (wide)
	// Output canvas frustum + pixel dims (single mode: == dominant eye)
	float projc_left, projc_right, projc_top, projc_bottom;
	unsigned int canvas_w, canvas_h;

	// Auto pose-delay state
	int stab_auto_delay;
	int stab_auto_want;
	uint32_t stab_auto_pending;

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

	context->texCrop.Reset();
	context->tex.Reset();
	// Abandon (do NOT Release) the previous mirror SRVs before re-acquiring:
	// the SteamVR runtime owns and caches them, and a plain Release()
	// mid-session corrupts its cache - crashed in d3d11.dll on the settings
	// re-init path (2026-07-18). Detach only nulls our pointer, so a failed
	// GetMirrorTextureD3D11 below still can't masquerade as success.
	context->mirrorSrv.Detach();
	context->mirrorSrv2.Detach();

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
					vr::VR_Shutdown();
					tex2D.Reset();
					context->texCrop.Reset();
					return;
				}
				HANDLE handle = nullptr;
				HRESULT hrHandle = context->res->GetSharedHandle(&handle);
				if (FAILED(hrHandle)) {
					warn("win_openvr_show: GetSharedHandle failed");
					init_inprog = false;
					vr::VR_Shutdown();
					context->res.Reset();
					tex2D.Reset();
					context->texCrop.Reset();
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
						context->stabRTV.Reset();
						context->texCrop.Reset();
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

	if (context->texture) destroy_obs_texture(&context->texture);
	context->stabRTV.Reset();
	context->stabVS.Reset();
	context->stabPS.Reset();
	context->stabCB.Reset();
	context->stabSampler.Reset();
	context->stabRaster.Reset();
	context->stab_shader_ok = false;
	if (context->texCrop) context->texCrop.Reset();
	if (context->tex) context->tex.Reset();
	if (context->mirrorSrv) context->mirrorSrv.Reset();
	if (context->mirrorSrv2) context->mirrorSrv2.Reset();
	context->wide_active = false;
	if (context->res) context->res.Reset();
	if (context->shared_device) context->shared_device.Reset();
	if (context->shared_context) context->shared_context.Reset();

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

	// Only crop geometry / buffer-size changes require re-acquiring the
	// mirror texture; the stabilization filter parameters below apply live.
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

	bool stab_pos_comp = obs_data_get_bool(settings, "stab_pos_comp");
	if (stab_pos_comp != context->stab_pos_comp)
		context->stab_has_state = false; // reseed the position state cleanly
	context->stab_pos_comp = stab_pos_comp;
	context->stab_pos_depth = std::min(std::max(obs_data_get_double(settings, "stab_pos_depth"), 0.3), 20.0);

	int stab_pose_delay = std::min(std::max((int)obs_data_get_int(settings, "stab_pose_delay"), -1), 3);
	if (stab_pose_delay != context->stab_pose_delay) {
		context->stab_auto_pending = 0;
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
	obs_data_set_default_bool(settings, "stab_pos_comp", false);
	obs_data_set_default_double(settings, "stab_pos_depth", 1.5);
	obs_data_set_default_int(settings, "stab_smooth_ms", 500);
	obs_data_set_default_int(settings, "stab_filter", 0);
	obs_data_set_default_int(settings, "stab_pose_delay", -1); // Auto
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
	for (uint32_t k = 0; k <= 3; k++) {
		vr::Compositor_FrameTiming t = {};
		t.m_nSize = sizeof(vr::Compositor_FrameTiming);
		if (!vr::VRCompositor()->GetFrameTiming(&t, k))
			break;
		if (t.m_nNumFramePresents > 0) {
			want = (int)k;
			break;
		}
	}

	// Hysteresis: adopt a new offset only after ~30 consecutive frames of
	// the SAME candidate, then reseed the filter so the pose-time
	// discontinuity can't cause a lurch.
	if (want == context->stab_auto_delay) {
		context->stab_auto_pending = 0;
	} else if (want != context->stab_auto_want) {
		context->stab_auto_want = want;
		context->stab_auto_pending = 1;
	} else if (++context->stab_auto_pending >= 30) {
		context->stab_auto_delay = want;
		context->stab_auto_pending = 0;
		context->stab_has_state = false;
		if (context->stab_debug)
			info("stab: auto pose delay -> %d", want);
	}
	return context->stab_auto_delay;
}

// Fetch the raw pose + reprojection flag matching the current mirror frame.
static bool stab_fetch_pose(const vr::Compositor_FrameTiming *cur, int pose_delay, quatd *q_out, vec3d *p_out,
			    double *t_out, bool *frozen_out)
{
	const vr::TrackedDevicePose_t *pose = &cur->m_HmdPose;
	double pose_time = cur->m_flSystemTimeInSeconds;
	uint32_t reproj_flags = cur->m_nReprojectionFlags;

	// The newest timing entry is the frame the compositor just started;
	// the mirror texture shows an older, already-composited frame.
	vr::Compositor_FrameTiming past = {};
	if (pose_delay > 0) {
		past.m_nSize = sizeof(vr::Compositor_FrameTiming);
		if (vr::VRCompositor()->GetFrameTiming(&past, (uint32_t)pose_delay) &&
		    past.m_HmdPose.bPoseIsValid) {
			pose = &past.m_HmdPose;
			pose_time = past.m_flSystemTimeInSeconds;
			reproj_flags = past.m_nReprojectionFlags;
		}
	}

	if (!pose->bPoseIsValid || pose->eTrackingResult != vr::TrackingResult_Running_OK)
		return false;

	*q_out = quat_normalize(quat_from_hmd34(pose->mDeviceToAbsoluteTracking));
	const vr::HmdMatrix34_t &hm = pose->mDeviceToAbsoluteTracking;
	*p_out = {hm.m[0][3], hm.m[1][3], hm.m[2][3]};
	*t_out = pose_time;
	*frozen_out = (reproj_flags & vr::VRCompositor_ReprojectionMotion) != 0;
	return true;
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
		context->stab_has_state = true;
		return false;
	}

	const double dt = pose_time - context->prev_pose_time;
	context->prev_pose_time = pose_time;
	if (dt <= 0.0 || dt > 0.25) {
		context->q_smooth = q_a;
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
	const quatd q_target = context->stab_roll_lock ? quat_level_roll(q_a) : q_a;
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
	if (!stab_fetch_pose(cur, pose_delay, &q, &p, &t, &frozen)) {
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
	if (!context->stab_debug)
		return;
	if (fabs(corr_a) > context->dbg_max_dx)
		context->dbg_max_dx = fabs(corr_a);
	if (fabs(corr_b) > context->dbg_max_dy)
		context->dbg_max_dy = fabs(corr_b);
	if (clamped)
		context->dbg_clamped++;
	const double elapsed = context->prev_pose_time - context->dbg_window_start;
	if (elapsed >= 2.0) {
		info("stab: compositor=%.1f fps, tau=%.0f ms, max corr=(%.2f, %.2f) %s, clamped %u frames",
		     (double)(cur->m_nFrameIndex - context->dbg_window_frame_index) / elapsed,
		     context->stab_tau * 1000.0, context->dbg_max_dx, context->dbg_max_dy, unit, context->dbg_clamped);
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
				 bool frozen, bool clamped, double corr_deg, const vec3d &posc_eye)
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
			"time,frame,delay,frozen,clamped,qa_w,qa_x,qa_y,qa_z,qs_w,qs_x,qs_y,qs_z,"
			"pa_x,pa_y,pa_z,ps_x,ps_y,ps_z,corr_deg,posc_x,posc_y,posc_z\n");
		context->stab_telemetry_lines = 0;
	}
	const quatd &qa = context->stab_q_ref;
	const quatd &qs = context->q_smooth;
	fprintf(context->stab_telemetry_file,
		"%.6f,%u,%d,%d,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
		"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.6f,%.6f,%.6f\n",
		context->prev_pose_time, cur->m_nFrameIndex, pose_delay, frozen ? 1 : 0, clamped ? 1 : 0, qa.w, qa.x,
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
	int pose_delay = 0;

	if (context->stabilize) {
		// On motion-smoothed frames the pose pairing is unreliable, so keep
		// the previous correction instead of updating the filter with bad data.
		pose_delay = stab_resolve_pose_delay(context);
		quatd q_a;
		vec3d p_a;
		double t_a = 0.0;
		const bool have = stab_fetch_pose(cur, pose_delay, &q_a, &p_a, &t_a, &frozen);
		if (!have)
			context->stab_has_state = false;

		if (have && frozen && context->stab_has_state) {
			q_err = context->q_err_last;
			posc_h = context->stab_posc_last;
		} else if (have) {
			if (stab_filter_core(context, cur, q_a, p_a, t_a)) {
				q_err = quat_mul(quat_conj(q_a), context->q_smooth);
				posc_h = context->stab_posc;
				if (!context->stab_black_edges && !stab_corners_feasible(context, q_err, posc_h)) {
					// Clamp the correction to the largest feasible fraction and
					// absorb the rotational excess into the output pose
					// (anti-windup; the positional state is self-bounded).
					const quatd q_target = context->q_smooth;
					const vec3d posc_full = posc_h;
					double lo = 0.0, hi = 1.0;
					for (int it = 0; it < 12; it++) {
						const double mid = 0.5 * (lo + hi);
						const quatd q_try =
							quat_mul(quat_conj(q_a), quat_slerp(q_a, q_target, mid));
						const vec3d p_try = {posc_full.x * mid, posc_full.y * mid,
								     posc_full.z * mid};
						if (stab_corners_feasible(context, q_try, p_try))
							lo = mid;
						else
							hi = mid;
					}
					context->q_smooth = quat_normalize(quat_slerp(q_a, q_target, lo));
					q_err = quat_mul(quat_conj(q_a), context->q_smooth);
					posc_h = {posc_full.x * lo, posc_full.y * lo, posc_full.z * lo};
					clamped = true;
				}
			}
		}
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
	ID3D11ShaderResourceView *srvs[2] = {context->mirrorSrv.Get(),
					     context->wide_active ? context->mirrorSrv2.Get()
								  : context->mirrorSrv.Get()};
	ctx->PSSetShaderResources(0, 2, srvs);
	ID3D11SamplerState *smp = context->stabSampler.Get();
	ctx->PSSetSamplers(0, 1, &smp);
	ctx->RSSetState(context->stabRaster.Get());
	ctx->OMSetBlendState(nullptr, nullptr, 0xffffffff);
	ctx->OMSetDepthStencilState(nullptr, 0);
	ctx->Draw(3, 0);
	ID3D11ShaderResourceView *nullsrvs[2] = {nullptr, nullptr};
	ctx->PSSetShaderResources(0, 2, nullsrvs);
	ID3D11RenderTargetView *nullrtv = nullptr;
	ctx->OMSetRenderTargets(1, &nullrtv, nullptr);

	if (context->stabilize) {
		const double corr_deg = 2.0 * acos(std::min(1.0, fabs(q_err.w))) * 57.29577951308232;
		const double pos_mm = sqrt(posc_h.x * posc_h.x + posc_h.y * posc_h.y + posc_h.z * posc_h.z) *
				      context->stab_pos_depth * 1000.0;
		stab_debug_stats(context, cur, corr_deg, pos_mm, clamped, "deg/mm");
		stab_telemetry_write(context, cur, pose_delay, frozen, clamped, corr_deg, pA);
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
	bool changed = set_vis(props, "stab_zoom_warning", on && nomargin);
	changed |= set_vis(props, "stab_preset", on);
	changed |= set_vis(props, "stab_filter", on);
	changed |= set_vis(props, "stab_roll_lock", on);
	changed |= set_vis(props, "stab_pos_comp", on);
	changed |= set_vis(props, "stab_pos_depth", on && pos);
	changed |= set_vis(props, "stab_smooth_ms", on && custom);
	changed |= set_vis(props, "stab_pose_delay", on);
	changed |= set_vis(props, "stab_debug", on);
	changed |= set_vis(props, "stab_telemetry", on);

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
	p = obs_properties_add_list(props, "stab_preset", obs_module_text("Stabilization Preset"), OBS_COMBO_TYPE_LIST,
				    OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(p, obs_module_text("Low"), 1);
	obs_property_list_add_int(p, obs_module_text("Medium"), 2);
	obs_property_list_add_int(p, obs_module_text("High"), 3);
	obs_property_list_add_int(p, obs_module_text("Custom"), 0);
	obs_property_set_modified_callback(p, stab_ui_modd);
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
	p = obs_properties_add_bool(props, "stab_roll_lock", obs_module_text("Roll Lock (level horizon)"));
	obs_property_set_long_description(
		p, obs_module_text("Keeps the horizon gravity-level instead of following smoothed head roll."));
	p = obs_properties_add_bool(props, "stab_pos_comp", obs_module_text("Positional Jitter Compensation (Experimental)"));
	obs_property_set_long_description(
		p, obs_module_text("Counteracts small positional shake (tremor, headset wobble) at an assumed "
				   "scene depth. Capped at 2 cm so deliberate movement is unaffected."));
	obs_property_set_modified_callback(p, stab_ui_modd);
	p = obs_properties_add_float_slider(props, "stab_pos_depth", obs_module_text("Reference Depth (m)"), 0.5, 20.0, 0.1);
	obs_property_set_long_description(
		p, obs_module_text("Scene distance that gets steadied perfectly; content at other depths keeps "
				   "some parallax shake. 1.0-2.0 m suits close-range games; use 10-20 m for "
				   "distant scenery / horizons (the positional correction shrinks with depth, "
				   "so far content stops being pushed around)."));
	p = obs_properties_add_int_slider(props, "stab_smooth_ms", obs_module_text("Smoothing Time (ms)"), 50, 2000, 10);
	obs_property_set_long_description(
		p, obs_module_text("Time constant of the head-direction average the view is reprojected to. "
				   "Higher = steadier output that trails further behind deliberate turns. "
				   "Presets: Low 200, Medium 500, High 1000."));
	p = obs_properties_add_list(props, "stab_pose_delay", obs_module_text("Stabilization Pose Delay"),
				    OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	obs_property_list_add_int(p, obs_module_text("Auto"), -1);
	obs_property_list_add_int(p, "0", 0);
	obs_property_list_add_int(p, "1", 1);
	obs_property_list_add_int(p, "2", 2);
	obs_property_list_add_int(p, "3", 3);
	obs_property_set_long_description(
		p, obs_module_text("Which past compositor frame the pose lookup pairs with the mirror image. "
				   "Auto tracks the last presented frame and adapts per session; "
				   "pick a fixed value only if Auto misbehaves."));
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
