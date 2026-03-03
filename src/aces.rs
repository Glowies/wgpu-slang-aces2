use glam::{Mat3, Vec3};

// CAM Parameters
const ref_luminance:f32 = 100.;
const L_A:f32 = 100.;
const Y_b:f32 = 20.;
const surround:Vec3 = Vec3::new(0.9, 0.59, 0.9); // Dim surround

const J_scale:f32 = 100.0;
const cam_nl_Y_reference:f32 = 100.0;
const cam_nl_offset:f32 = 0.2713 * cam_nl_Y_reference;
const cam_nl_scale:f32 = 4.0 * cam_nl_Y_reference;

// Chroma compression
const chroma_compress:f32 = 2.4;
const chroma_compress_fact:f32 = 3.3;
const chroma_expand:f32 = 1.3;
const chroma_expand_fact:f32 = 0.69;
const chroma_expand_thr:f32 = 0.5;

// Gamut compression
const smooth_cusps:f32 = 0.12;
const smooth_m:f32 = 0.27;
const cusp_mid_blend:f32 = 1.3;

const focus_gain_blend:f32 = 0.3;
const focus_adjust_gain:f32 = 0.55;
const focus_distance:f32 = 1.35;
const focus_distance_scaling:f32 = 1.75;

const compression_threshold:f32 = 0.75;

const MATRIX_IDENTITY:Mat3 = Mat3::IDENTITY;

fn invert_f33(m: Mat3) -> Mat3 {
    m.inverse()
}

fn scale_matrix_diagonal_f33_f3(M: Mat3, d: Vec3) -> Mat3{
    let mut R = M;
    R.x_axis.x *= d.x;
    R.y_axis.y *= d.y;
    R.z_axis.z *= d.z;
    return R;
}

fn mult_f33_f33(A: Mat3, B: Mat3) -> Mat3 { return A * B; }

fn mult_f3_f33(A: Vec3, B: Mat3) -> Vec3 { return B * A; }

fn mult_f_f33(A: f32, B: Mat3) -> Mat3 { return A * B; }

fn mult_f_f3(A: f32, B: Vec3) -> Vec3 { return A * B; }

fn lerp_f3(A: Vec3, B: Vec3, t: f32) -> Vec3 { A.lerp(B, t) }

fn f3_from_f(A: f32) -> Vec3 { return Vec3::ONE * A; }

fn radians_to_degrees(A: f32) -> f32 { A.to_degrees() }

fn degrees_to_radians(A: f32) -> f32 { A.to_radians() }

fn copysign(A: f32, B: f32) -> f32 { if B < 0.0 { -A.abs() } else { A.abs() } }

fn colmajor_to_rowmajor(A: Mat3) -> Mat3 { A.transpose() }

struct TSParams {
    n: f32,
    n_r: f32,
    g: f32,
    t_1: f32,
    c_t: f32,
    s_2: f32,
    u_2: f32,
    m_2: f32,
    forward_limit: f32,
    inverse_limit: f32,
    log_peak: f32,
}

struct JMhParams {
    // Pre-computed conversion matrices and constants for conversions to/from JMh
    matrix_rgb_to_cam16_c: Mat3,
    matrix_cam16_c_to_rgb: Mat3,
    matrix_cone_response_to_aab: Mat3,
    matrix_aab_to_cone_response: Mat3,
    f_l_n: f32, // F_L normalised
    cz: f32,
    inv_cz: f32, // 1/cz
    a_w_j: f32,
    inv_a_w_j: f32, // 1/A_w_J
}

struct ODTParams {
    peak_luminance: f32,

    // JMh parameters
    input_params: JMhParams,
    reach_params: JMhParams,
    limit_params: JMhParams,

    // Tonescale parameters
    ts: TSParams,

    // Shared compression parameters
    limit_j_max: f32,
    model_gamma_inv: f32,

    // Chroma compression parameters
    sat: f32,
    sat_thr: f32,
    compr: f32,
    chroma_compress_scale: f32,

    // Gamut compression parameters
    mid_j: f32,
    focus_dist: f32,
    lower_hull_gamma_inv: f32,
    hue_linearity_search_range: [i32; 2],
}

// Tonescale pre-calculations
fn init_tsparams(peak_luminance: f32) -> TSParams {
    // Preset constants that set the desired behavior for the curve
    let n: f32 = peak_luminance;

    let n_r: f32 = 100.0; // normalized white in nits (what 1.0 should be)
    let g: f32 = 1.15; // surround / contrast
    let c: f32 = 0.18; // anchor for 18% grey
    let c_d: f32 = 10.013; // output luminance of 18% grey (in nits)
    let w_g: f32 = 0.14; // change in grey between different peak luminance
    let t_1: f32 = 0.04; // shadow toe or flare/glare compensation
    let r_hit_min: f32 = 128.; // scene-referred value "hitting the roof" for SDR (e.g. when n = 100 nits)
    let r_hit_max: f32 = 896.; // scene-referred value "hitting the roof" for when n = 10000 nits

    // Calculate output constants
    let r_hit: f32 =
        r_hit_min + (r_hit_max - r_hit_min) * ((n / n_r).ln() / (10000.0f32 / 100.0f32).ln());
    let m_0: f32 = n / n_r;
    let m_1: f32 = 0.5 * (m_0 + (m_0 * (m_0 + 4. * t_1)).sqrt());
    let u: f32 = ((r_hit / m_1) / ((r_hit / m_1) + 1.0)).powf(g);
    let m: f32 = m_1 / u;
    let w_i: f32 = (n / 100.).ln() / (2.0f32).ln();
    let c_t: f32 = c_d / n_r * (1. + w_i * w_g);
    let g_ip: f32 = 0.5 * (c_t + (c_t * (c_t + 4. * t_1)).sqrt());
    let g_ipp2: f32 = -(m_1 * (g_ip / m).powf(1. / g)) / ((g_ip / m).powf(1. / g) - 1.);
    let w_2: f32 = c / g_ipp2;
    let s_2: f32 = w_2 * m_1;
    let u_2: f32 = ((r_hit / m_1) / ((r_hit / m_1) + w_2)).powf(g);
    let m_2: f32 = m_1 / u_2;

    TSParams {
        n,
        n_r,
        g,
        t_1,
        c_t,
        s_2,
        u_2,
        m_2,
        forward_limit: 8.0 * r_hit,
        inverse_limit: n / (u_2 * n_r),
        log_peak: (n / n_r).log10(),
    }
}

fn init_JMhParams(rgb_to_xyz: Mat3) -> JMhParams
{
    let MATRIX_16 = Mat3::from_cols_array(&[
    	0.401288, 0.650173, -0.051461,
        -0.250268, 1.204414, 0.045854,
    	-0.002079, 0.048952, 0.953127
    ]);

    let base_cone_response_to_Aab = Mat3::from_cols_array(&[
        2., 1., 1. / 9.,
        1., -12. / 11., 1. / 9.,
        1. / 20., 1. / 11., -2. / 9.]);

    let RGB_TO_XYZ = rgb_to_xyz;
    let XYZ_w = mult_f3_f33(f3_from_f(ref_luminance), RGB_TO_XYZ);

    let Y_w = XYZ_w[1];

    // Step 0 - Converting CIE XYZ tristimulus values to sharpened RGB values
    let RGB_w = mult_f3_f33(XYZ_w, MATRIX_16);

    // Viewing condition dependent parameters
    let k = 1. / (5. * L_A + 1.);
    let k4 = k * k * k * k;
    let F_L = 0.2 * k4 * (5. * L_A) + 0.1 * (1. - k4).powf(2.) * (5. * L_A).powf(1. / 3.);

    let F_L_n = F_L / ref_luminance;
    let model_gamma = surround[1] * (1.48 + (Y_b / ref_luminance).sqrt());
    let cz = model_gamma;

    let D_RGB = Vec3::new(
        F_L_n * Y_w / RGB_w[0],
        F_L_n * Y_w / RGB_w[1],
        F_L_n * Y_w / RGB_w[2]);

    let RGB_wc = Vec3::new(
        D_RGB[0] * RGB_w[0],
        D_RGB[1] * RGB_w[1],
        D_RGB[2] * RGB_w[2]);

    let RGB_Aw = Vec3::new(
        post_adaptation_cone_response_compression_fwd(RGB_wc[0]),
        post_adaptation_cone_response_compression_fwd(RGB_wc[1]),
        post_adaptation_cone_response_compression_fwd(RGB_wc[2]));

    let cone_response_to_Aab = mult_f33_f33(mult_f_f33(cam_nl_scale, MATRIX_IDENTITY), base_cone_response_to_Aab);
    let cone_response_to_Aab_array = cone_response_to_Aab.to_cols_array_2d();
    let A_w = cone_response_to_Aab_array[0][0] * RGB_Aw[0] + cone_response_to_Aab_array[1][0] * RGB_Aw[1] + cone_response_to_Aab_array[2][0] * RGB_Aw[2];
    let A_w_J = _post_adaptation_cone_response_compression_fwd(F_L);

    // Prescale the CAM16 LMS responses to directly provide for chromatic adaptation
    let M1 = mult_f33_f33(RGB_TO_XYZ, MATRIX_16);
    let M2 = mult_f_f33(ref_luminance, MATRIX_IDENTITY);
    let MATRIX_RGB_to_CAM16 = mult_f33_f33(M1, M2);
    let MATRIX_RGB_to_CAM16_c = mult_f33_f33(MATRIX_RGB_to_CAM16, scale_matrix_diagonal_f33_f3(MATRIX_IDENTITY, D_RGB));

    let MATRIX_cone_response_to_Aab = Mat3::from_cols_array(&[
        cone_response_to_Aab_array[0][0] / A_w, cone_response_to_Aab_array[0][1] * 43. * surround[2], cone_response_to_Aab_array[0][2] * 43. * surround[2],
        cone_response_to_Aab_array[1][0] / A_w, cone_response_to_Aab_array[1][1] * 43. * surround[2], cone_response_to_Aab_array[1][2] * 43. * surround[2],
        cone_response_to_Aab_array[2][0] / A_w, cone_response_to_Aab_array[2][1] * 43. * surround[2], cone_response_to_Aab_array[2][2] * 43. * surround[2]]);

    JMhParams {
        matrix_rgb_to_cam16_c: MATRIX_RGB_to_CAM16_c,
        matrix_cam16_c_to_rgb: invert_f33(MATRIX_RGB_to_CAM16_c),
        matrix_cone_response_to_aab: MATRIX_cone_response_to_Aab,
        matrix_aab_to_cone_response: invert_f33(MATRIX_cone_response_to_Aab),
        f_l_n: F_L_n,
        cz: cz,
        inv_cz: 1. / cz,
        a_w_j: A_w_J,
        inv_a_w_j: 1. / A_w_J,
    }
}

fn init_ODTParams(float peakLuminance,
                         Chromaticities limitingPrimaries) -> ODTParams
{
    ODTParams p;

    p.peakLuminance = peakLuminance;

    // JMh parameters
    p.input_params = init_JMhParams(AP0);
    p.reach_params = init_JMhParams(REACH_PRI);
    p.limit_params = init_JMhParams(limitingPrimaries);

    // Tonescale parameters
    p.ts = init_TSParams(peakLuminance);

    // Shared compression paramters
    p.limit_J_max = Y_to_J(peakLuminance, p.input_params);
    p.model_gamma_inv = 1. / model_gamma;
    p.TABLE_reach_M = make_reach_m_table(p.reach_params, p.limit_J_max);

    // Chroma compression parameters
    p.sat = max(0.2, chroma_expand - (chroma_expand * chroma_expand_fact) * p.ts.log_peak);
    p.sat_thr = chroma_expand_thr / peakLuminance;
    p.compr = chroma_compress + (chroma_compress * chroma_compress_fact) * p.ts.log_peak;
    p.chroma_compress_scale = pow(0.03379 * peakLuminance, 0.30596) - 0.45135;

    // Gamut compression parameters
    p.mid_J = Y_to_J(p.ts.c_t * ref_luminance, p.input_params);
    p.focus_dist = focus_distance + focus_distance * focus_distance_scaling * p.ts.log_peak;
    const float lower_hull_gamma = 1.14 + 0.07 * p.ts.log_peak;
    p.lower_hull_gamma_inv = 1. / lower_hull_gamma;
    p.TABLE_gamut_cusps = make_uniform_hue_gamut_table(p.reach_params, p.limit_params, p);
    for (int i = 0; i != totalTableSize; i = i + 1)
    {
        p.TABLE_hues[i] = p.TABLE_gamut_cusps[i][2];
    }
    p.TABLE_upper_hull_gamma = make_upper_hull_gamma_table(p.TABLE_gamut_cusps, p);
    p.hue_linearity_search_range = determine_hue_linearity_search_range(p.TABLE_hues);

    return p;
}
