
// SOURCE: https://raw.githubusercontent.com/aces-aswf/aces-core/refs/heads/dev/lib/Lib.Academy.Tonescale.ctl

// <ACEStransformID>urn:ampas:aces:transformId:v2.0:Lib.Academy.Tonescale.a2.v1</ACEStransformID>
// <ACESuserName>Tonescale Functions</ACESuserName>

// 
// Library File with functions used for pre-calculating the tonescale parameters and 
// applying the fwd/inv tonescale function
//

struct TSParams 
{
	float n;
	float n_r;
	float g;
	float t_1;
	float c_t; 
	float s_2;
	float u_2;
	float m_2;
	float forward_limit;
	float inverse_limit;
	float log_peak;
};

/* --- Tone scale math --- */
float tonescale_fwd(float x,		// scene-referred input (i.e. linear ACES2065-1)
					TSParams params // struct of type TSParams
)
{
	// forward MM tone scale
	float f = params.m_2 * pow(max(0.0, x) / (x + params.s_2), params.g);
	float h = max(0., f * f / (f + params.t_1)); // forward flare

	return h * params.n_r; // output is luminance in cd/m^2
}

float tonescale_inv(float Y,		// luminance in cd/m^2
					TSParams params // struct of type TSParams
)
{
	float Z = max(0., min(params.n / (params.u_2 * params.n_r), Y));
	float h = (Z + sqrt(Z * (4. * params.t_1 + Z))) / 2.;
	float f = params.s_2 / (pow((params.m_2 / h), (1. / params.g)) - 1.);

	return f; // output is scene-referred input
}
