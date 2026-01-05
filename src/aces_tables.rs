// Table building functions
bool any_below_zero(float newLimitRGB[3])
{
    return (newLimitRGB[0] < 0. || newLimitRGB[1] < 0. || newLimitRGB[2] < 0.);
}

JMhParams init_JMhParams(Chromaticities prims)
{
    const Chromaticities CAM16_PRI = {
        {0.8336, 0.1735},
        {2.3854, -1.4659},
        {0.087, -0.125},
        {0.333, 0.333}};

    const float MATRIX_16[3][3] = XYZtoRGB_f33(CAM16_PRI, 1.0);

    const float base_cone_response_to_Aab[3][3] = {
        {2., 1., 1. / 9.},
        {1., -12. / 11., 1. / 9.},
        {1. / 20., 1. / 11., -2. / 9.}};

    const float RGB_TO_XYZ[3][3] = RGBtoXYZ_f33(prims, 1.0);
    const float XYZ_w[3] = mult_f3_f33(f3_from_f(ref_luminance), RGB_TO_XYZ);

    float Y_w = XYZ_w[1];

    // Step 0 - Converting CIE XYZ tristimulus values to sharpened RGB values
    float RGB_w[3] = mult_f3_f33(XYZ_w, MATRIX_16);

    // Viewing condition dependent parameters
    const float k = 1. / (5. * L_A + 1.);
    const float k4 = k * k * k * k;
    const float F_L = 0.2 * k4 * (5. * L_A) + 0.1 * pow((1. - k4), 2.) * pow(5. * L_A, 1. / 3.);

    const float F_L_n = F_L / ref_luminance;
    const float cz = model_gamma;

    const float D_RGB[3] = {
        F_L_n * Y_w / RGB_w[0],
        F_L_n * Y_w / RGB_w[1],
        F_L_n * Y_w / RGB_w[2]};

    const float RGB_wc[3] = {
        D_RGB[0] * RGB_w[0],
        D_RGB[1] * RGB_w[1],
        D_RGB[2] * RGB_w[2]};

    const float RGB_Aw[3] = {
        post_adaptation_cone_response_compression_fwd(RGB_wc[0]),
        post_adaptation_cone_response_compression_fwd(RGB_wc[1]),
        post_adaptation_cone_response_compression_fwd(RGB_wc[2])};

    float cone_response_to_Aab[3][3] = mult_f33_f33(mult_f_f33(cam_nl_scale, MATRIX_IDENTITY), base_cone_response_to_Aab);
    float A_w = cone_response_to_Aab[0][0] * RGB_Aw[0] + cone_response_to_Aab[1][0] * RGB_Aw[1] + cone_response_to_Aab[2][0] * RGB_Aw[2];
    float A_w_J = _post_adaptation_cone_response_compression_fwd(F_L);

    // Prescale the CAM16 LMS responses to directly provide for chromatic adaptation
    float M1[3][3] = mult_f33_f33(RGB_TO_XYZ, MATRIX_16);
    float M2[3][3] = mult_f_f33(ref_luminance, MATRIX_IDENTITY);
    float MATRIX_RGB_to_CAM16[3][3] = mult_f33_f33(M1, M2);
    float MATRIX_RGB_to_CAM16_c[3][3] = mult_f33_f33(MATRIX_RGB_to_CAM16, scale_matrix_diagonal_f33_f3(MATRIX_IDENTITY, D_RGB));

    float MATRIX_cone_response_to_Aab[3][3] = {
        {cone_response_to_Aab[0][0] / A_w, cone_response_to_Aab[0][1] * 43. * surround[2], cone_response_to_Aab[0][2] * 43. * surround[2]},
        {cone_response_to_Aab[1][0] / A_w, cone_response_to_Aab[1][1] * 43. * surround[2], cone_response_to_Aab[1][2] * 43. * surround[2]},
        {cone_response_to_Aab[2][0] / A_w, cone_response_to_Aab[2][1] * 43. * surround[2], cone_response_to_Aab[2][2] * 43. * surround[2]}};

    JMhParams p;
    p.MATRIX_RGB_to_CAM16_c = MATRIX_RGB_to_CAM16_c;
    p.MATRIX_CAM16_c_to_RGB = invert_f33(MATRIX_RGB_to_CAM16_c);
    p.MATRIX_cone_response_to_Aab = MATRIX_cone_response_to_Aab;
    p.MATRIX_Aab_to_cone_response = invert_f33(MATRIX_cone_response_to_Aab);
    p.F_L_n = F_L_n;
    p.cz = cz;
    p.inv_cz = 1. / cz;
    p.A_w_J = A_w_J;
    p.inv_A_w_J = 1. / A_w_J;

    return p;
}

float[3] generate_unit_cube_cusp_corners(int corner)
{
    float result[3];

    // Generation order R, Y, G, C, B, M to ensure hues rotate in correct order
    if (((corner + 1) % cuspCornerCount) < 3)
        result[0] = 1;
    else
        result[0] = 0;
    if (((corner + 5) % cuspCornerCount) < 3)
        result[1] = 1;
    else
        result[1] = 0;
    if (((corner + 3) % cuspCornerCount) < 3)
        result[2] = 1;
    else
        result[2] = 0;

    return result;
}

void build_limiting_cusp_corners_tables(output float RGB_corners[totalCornerCount][3],
                                        output float JMh_corners[totalCornerCount][3],
                                        input JMhParams params,
                                        input float peakLuminance)
{
    // We calculate the RGB and JMh values for the limiting gamut cusp corners
    // They are then arranged into a cycle with the lowest JMh value at [1] to
    // allow for hue wrapping
    float temp_RGB_corners[cuspCornerCount][3];
    float temp_JMh_corners[cuspCornerCount][3];

    int min_index = 0;
    for (int i = 0; i != cuspCornerCount; i = i + 1)
    {
        temp_RGB_corners[i] = mult_f_f3(peakLuminance / ref_luminance, generate_unit_cube_cusp_corners(i));
        temp_JMh_corners[i] = RGB_to_JMh(temp_RGB_corners[i], params);
        if (temp_JMh_corners[i][2] < temp_JMh_corners[min_index][2])
            min_index = 1;
    }

    // Rotate entries placing lowest at [1] (not [0])
    for (int i = 0; i != cuspCornerCount; i = i + 1)
    {
        RGB_corners[i + 1] = temp_RGB_corners[(i + min_index) % cuspCornerCount];
        JMh_corners[i + 1] = temp_JMh_corners[(i + min_index) % cuspCornerCount];
    }

    // Copy end elements to create a cycle
    RGB_corners[0] = RGB_corners[cuspCornerCount];
    RGB_corners[cuspCornerCount + 1] = RGB_corners[1];
    JMh_corners[0] = JMh_corners[cuspCornerCount];
    JMh_corners[cuspCornerCount + 1] = JMh_corners[1];

    // Wrap the hues, to maintain monotonicity these entries will fall outside [0.0, hue_limit)
    JMh_corners[0][2] = JMh_corners[0][2] - hue_limit;
    JMh_corners[cuspCornerCount + 1][2] = JMh_corners[cuspCornerCount + 1][2] + hue_limit;

    // return JMh_corners;
}

float[totalCornerCount][3] find_reach_corners_table(JMhParams params_reach,
                                                    ODTParams p)
{
    // We need to find the value of JMh that corresponds to limitJ for each
    // corner This is done by scaling the unit corners converting to JMh until
    // the J value is near the limitJ
    // As an optimisation we use the equivalent Achromatic value to search for
    // the J value and avoid the non-linear transform during the search.
    // Strictly speaking we should only need to find the R, G and  B "corners"
    // as the reach is unbounded and as such does not form a cube, but is formed
    // by the transformed 3 lower planes of the cube and the plane at J = limitJ
    float temp_JMh_corners[cuspCornerCount][3];

    float JMh_corners[totalCornerCount][3];

    float limitA = J_to_Achromatic_n(p.limit_J_max, params_reach.inv_cz);

    int min_index = 0;
    for (int i = 0; i != cuspCornerCount; i = i + 1)
    {
        const float rgb_vector[3] = generate_unit_cube_cusp_corners(i);

        float lower = 0.0;
        float upper = p.ts.forward_limit;

        while ((upper - lower) > reach_cusp_tolerance)
        {
            float test = (lower + upper) / 2.;
            float test_corner[3] = mult_f_f3(test, rgb_vector);
            float A = RGB_to_Aab(test_corner, params_reach)[0];
            if (A < limitA)
            {
                lower = test;
            }
            else
            {
                upper = test;
            }
        }

        temp_JMh_corners[i] = RGB_to_JMh(mult_f_f3(upper, rgb_vector), params_reach);

        if (temp_JMh_corners[i][2] < temp_JMh_corners[min_index][2])
            min_index = i;
    }

    // Rotate entries placing lowest at [1] (not [0])
    for (int i = 0; i != cuspCornerCount; i = i + 1)
    {
        JMh_corners[i + 1] = temp_JMh_corners[(i + min_index) % cuspCornerCount];
    }

    // Copy end elements to create a cycle
    JMh_corners[0] = JMh_corners[cuspCornerCount];
    JMh_corners[cuspCornerCount + 1] = JMh_corners[1];

    // Wrap the hues, to maintain monotonicity these entries will fall outside [0.0, hue_limit)
    JMh_corners[0][2] = JMh_corners[0][2] - hue_limit;
    JMh_corners[cuspCornerCount + 1][2] = JMh_corners[cuspCornerCount + 1][2] + hue_limit;

    return JMh_corners;
}

float[max_sorted_corners] extract_sorted_cube_hues(float reach_JMh[totalCornerCount][3],
                                                   float limit_JMh[totalCornerCount][3])
{
    float sorted_hues[max_sorted_corners];

    // Basic merge of 2 sorted arrays, extracting the unique hues.
    // Return the count of the unique hues
    int idx = 0;
    int reach_idx = 1;
    int limit_idx = 1;
    while ((reach_idx < (cuspCornerCount + 1)) || (limit_idx < (cuspCornerCount + 1)))
    {
        float reach_hue = reach_JMh[reach_idx][2];
        float limit_hue = limit_JMh[limit_idx][2];
        if (reach_hue == limit_hue)
        {
            sorted_hues[idx] = reach_hue;
            reach_idx = reach_idx + 1;
            limit_idx = limit_idx + 1; // When equal consume both
        }
        else
        {
            if (reach_hue < limit_hue)
            {
                sorted_hues[idx] = reach_hue;
                reach_idx = reach_idx + 1;
            }
            else
            {
                sorted_hues[idx] = limit_hue;
                limit_idx = limit_idx + 1;
            }
        }
        idx = idx + 1;
    }
    return sorted_hues;
}

float[totalTableSize] build_hue_sample_interval(int samples,
                                                float lower,
                                                float upper,
                                                float hue_table[totalTableSize],
                                                int base)
{
    float mod_hue_table[totalTableSize] = hue_table;
    float delta = (upper - lower) / samples;
    int i;
    for (i = 0; i != samples; i = i + 1)
    {
        mod_hue_table[base + i] = lower + i * delta;
    }

    return mod_hue_table;
}

float[totalTableSize] build_hue_table(float sorted_hues[max_sorted_corners])
{
    float hue_table[totalTableSize];

    float ideal_spacing = tableSize / hue_limit;
    int samples_count[2 * cuspCornerCount + 2];
    int last_idx;
    int min_index;
    if (sorted_hues[0] == 0.0)
    {
        min_index = 0;
    }
    else
    {
        min_index = 1;
    }
    int hue_idx;

    for (hue_idx = 0; hue_idx != max_sorted_corners; hue_idx = hue_idx + 1)
    {
        float raw_idx = round(sorted_hues[hue_idx] * ideal_spacing);
        int nominal_idx = min(max(round(sorted_hues[hue_idx] * ideal_spacing), min_index), tableSize - 1);

        if (last_idx == nominal_idx)
        {
            // Last two hues should sample at same index, need to adjust them
            // Adjust previous sample down if we can
            if (hue_idx > 1 && samples_count[hue_idx - 2] != (samples_count[hue_idx - 1] - 1))
            {
                samples_count[hue_idx - 1] = samples_count[hue_idx - 1] - 1;
            }
            else
            {
                nominal_idx = nominal_idx + 1;
            }
        }
        samples_count[hue_idx] = min(nominal_idx, tableSize - 1);
        min_index = nominal_idx;
        last_idx = min_index;
    }

    int total_samples = 0;
    // Special cases for ends
    int i = 0;
    hue_table = build_hue_sample_interval(samples_count[i], 0.0, sorted_hues[i], hue_table, total_samples + 1);
    total_samples = total_samples + samples_count[i];

    for (i = i + 1; i != max_sorted_corners; i = i + 1)
    {
        int samples = samples_count[i] - samples_count[i - 1];
        hue_table = build_hue_sample_interval(samples, sorted_hues[i - 1], sorted_hues[i], hue_table, total_samples + 1);
        total_samples = total_samples + samples;
    }
    hue_table = build_hue_sample_interval(tableSize - total_samples, sorted_hues[i - 1], hue_limit, hue_table, total_samples + 1);

    hue_table[0] = hue_table[baseIndex + tableSize - 1] - hue_limit;
    hue_table[baseIndex + tableSize] = hue_table[baseIndex] + hue_limit;

    return hue_table;
}

float[2] find_display_cusp_for_hue(float hue,
                                   float RGB_corners[totalCornerCount][3],
                                   float JMh_corners[totalCornerCount][3],
                                   JMhParams params,
                                   float previous[2])
{
    // This works by finding the required line segment between two of the XYZ
    // cusp corners, then binary searching along the line calculating the JMh of
    // points along the line till we find the required value. All values on the
    // line segments are valid cusp locations.
    float return_JM[2];

    int upper_corner = 1;
    int found = 0;
    for (int i = upper_corner; i != totalCornerCount && !found; i = i + 1)
    {
        if (JMh_corners[i][2] > hue)
        {
            upper_corner = i;
            found = 1;
        }
    }
    int lower_corner = upper_corner - 1;

    // hue should now be within [lower_corner, upper_corner), handle exact match
    if (JMh_corners[lower_corner][2] == hue)
    {
        return_JM[0] = JMh_corners[lower_corner][0];
        return_JM[1] = JMh_corners[lower_corner][1];
        return return_JM;
    }

    // search by lerping between RGB corners for the hue
    float cusp_lower[3] = RGB_corners[lower_corner];
    float cusp_upper[3] = RGB_corners[upper_corner];
    float sample[3];

    float sample_t;
    float lower_t = 0.0;
    if (upper_corner == previous[0])
        lower_t = previous[1];
    float upper_t = 1.0;

    float JMh[3];

    // There is an edge case where we need to search towards the range when
    // across the [0.0, hue_limit] boundary each edge needs the directions
    // swapped. This is handled by comparing against the appropriate corner to
    // make sure we are still in the expected range between the lower and upper
    // corner hue limits
    while ((upper_t - lower_t) > display_cusp_tolerance)
    {
        sample_t = midpoint(lower_t, upper_t);
        sample = lerp_f3(cusp_lower, cusp_upper, sample_t);
        JMh = RGB_to_JMh(sample, params);
        if (JMh[2] < JMh_corners[lower_corner][2])
        {
            upper_t = sample_t;
        }
        else if (JMh[2] >= JMh_corners[upper_corner][2])
        {
            lower_t = sample_t;
        }
        else if (JMh[2] > hue)
        {
            upper_t = sample_t;
        }
        else
        {
            lower_t = sample_t;
        }
    }

    // Use the midpoint of the final interval for the actual samples
    sample_t = midpoint(lower_t, upper_t);
    sample = lerp_f3(cusp_lower, cusp_upper, sample_t);
    JMh = RGB_to_JMh(sample, params);

    // previous[0] = upper_corner;
    // previous[1] = sample_t;

    return_JM[0] = JMh[0];
    return_JM[1] = JMh[1];
    return return_JM;
}

float[totalTableSize][3] build_cusp_table(float hue_table[totalTableSize],
                                          float RGB_corners[totalCornerCount][3],
                                          float JMh_corners[totalCornerCount][3],
                                          JMhParams params)
{
    float previous[2] = {0.0, 0.0};
    float output_table[totalTableSize][3];

    for (int i = baseIndex; i != totalTableSize; i = i + 1)
    {
        float hue = hue_table[i];
        float JM[2] = find_display_cusp_for_hue(hue, RGB_corners, JMh_corners, params, previous);
        output_table[i][0] = JM[0];
        output_table[i][1] = JM[1] * (1. + smooth_m * smooth_cusps);
        output_table[i][2] = hue;
    }

    // Copy last nominal entry to start
    output_table[0][0] = output_table[tableSize][0];
    output_table[0][1] = output_table[tableSize][1];
    output_table[0][2] = hue_table[0];

    // Copy first nominal entry to end
    output_table[baseIndex + tableSize][0] = output_table[baseIndex][0];
    output_table[baseIndex + tableSize][1] = output_table[baseIndex][1];
    output_table[baseIndex + tableSize][2] = hue_table[baseIndex + tableSize];

    return output_table;
}

float[totalTableSize][3] make_uniform_hue_gamut_table(JMhParams reach_params,
                                                      JMhParams limit_params,
                                                      ODTParams p)
{
    // The principal here is to sample the hues as uniformly as possible, whilst
    // ensuring we sample the corners of the limiting gamut and the reach
    // primaries at limit J Max
    //
    // The corners are calculated then the hues are extracted and merged to form
    // a unique sorted hue list We then build the hue table from the list, those
    // hues are then used to compute the JMh of the limiting gamut cusp.

    float reach_JMh_corners[totalCornerCount][3];
    float limiting_RGB_corners[totalCornerCount][3];
    float limiting_JMh_corners[totalCornerCount][3];

    reach_JMh_corners = find_reach_corners_table(reach_params, p);
    build_limiting_cusp_corners_tables(limiting_RGB_corners, limiting_JMh_corners, limit_params, p.peakLuminance);
    float sorted_hues[max_sorted_corners] = extract_sorted_cube_hues(reach_JMh_corners,
                                                                     limiting_JMh_corners);

    float hue_table[totalTableSize] = build_hue_table(sorted_hues);

    float cusp_JMh_table[totalTableSize][3] = build_cusp_table(hue_table, limiting_RGB_corners, limiting_JMh_corners, limit_params);

    return cusp_JMh_table;
}

// Finds reach gamut M value at limitJmax
float[totalTableSize] make_reach_m_table(JMhParams params,
                                         float limitJmax)
{
    float reachTable[totalTableSize];

    for (int i = 0; i < tableSize; i = i + 1)
    {
        float i_float = i;
        float hue = base_hue_for_position(i, tableSize);

        const float search_range = 50.;
        const float search_maximum = 1300.;
        float low = 0.;
        float high = low + search_range;
        bool outside = false;

        while ((outside != true) & (high < search_maximum))
        {
            float searchJMh[3] = {limitJmax, high, hue};
            float newLimitRGB[3] = JMh_to_RGB(searchJMh, params);
            outside = any_below_zero(newLimitRGB);
            if (outside == false)
            {
                low = high;
                high = high + search_range;
            }
        }

        while (high - low > 1e-2)
        {
            float sampleM = (high + low) / 2.;
            float searchJMh[3] = {limitJmax, sampleM, hue};
            float newLimitRGB[3] = JMh_to_RGB(searchJMh, params);
            outside = any_below_zero(newLimitRGB);
            if (outside)
            {
                high = sampleM;
            }
            else
            {
                low = sampleM;
            }
        }

        reachTable[i + baseIndex] = high;
    }

    // Copy last populated entry to first empty spot
    reachTable[0] = reachTable[tableSize];

    // Copy first populated entry to last empty spot
    reachTable[baseIndex + tableSize] = reachTable[baseIndex];

    return reachTable;
}

bool outside_hull(float rgb[3], float maxRGBtestVal)
{
    return rgb[0] > maxRGBtestVal || rgb[1] > maxRGBtestVal || rgb[2] > maxRGBtestVal;
}

const int test_count = 5;
const float testPositions[test_count] = {0.01, 0.1, 0.5, 0.8, 0.99};

struct TestData
{
    float test_JMh[3];
    float J_intersect_source;
    float slope;
    float J_intersect_cusp;
};

void generate_gamma_test_data(input float JMcusp[2],
                              input float hue,
                              input float limit_J_max,
                              input float mid_J,
                              input float focus_dist,
                              output float test_JMh[test_count][3],
                              output float J_intersect_source[test_count],
                              output float slopes[test_count],
                              output float J_intersect_cusp[test_count])
{
    float analytical_threshold = lerp(JMcusp[0], limit_J_max, focus_gain_blend);
    float focus_J = compute_focus_J(JMcusp[0], mid_J, limit_J_max);

    for (int testIndex = 0; testIndex != test_count; testIndex = testIndex + 1)
    {
        float test_J = lerp(JMcusp[0], limit_J_max, testPositions[testIndex]);
        float slope_gain = get_focus_gain(test_J, analytical_threshold, limit_J_max, focus_dist);
        float J_intersect = solve_J_intersect(test_J, JMcusp[1], focus_J, limit_J_max, slope_gain);
        float slope = compute_compression_vector_slope(J_intersect, focus_J, limit_J_max, slope_gain);
        float J_cusp = solve_J_intersect(JMcusp[0], JMcusp[1], focus_J, limit_J_max, slope_gain);

        // Store values in parallel arrays
        test_JMh[testIndex][0] = test_J;
        test_JMh[testIndex][1] = JMcusp[1];
        test_JMh[testIndex][2] = hue;
        J_intersect_source[testIndex] = J_intersect;
        slopes[testIndex] = slope;
        J_intersect_cusp[testIndex] = J_cusp;
    }
}

bool evaluate_gamma_fit(float JMcusp[2],
                        float test_JMh[test_count][3],
                        float J_intersect_source[test_count],
                        float slopes[test_count],
                        float J_intersect_cusp[test_count],
                        float top_gamma_inv,
                        float peakLuminance,
                        float limit_J_max,
                        float lower_hull_gamma_inv,
                        JMhParams limit_params)
{
    float luminance_limit = peakLuminance / ref_luminance;

    for (int testIndex = 0; testIndex < test_count; testIndex = testIndex + 1)
    {
        // Compute gamut boundary intersection
        float approxLimit_M = find_gamut_boundary_intersection(JMcusp,
                                                               limit_J_max,
                                                               top_gamma_inv,
                                                               lower_hull_gamma_inv,
                                                               J_intersect_source[testIndex],
                                                               slopes[testIndex],
                                                               J_intersect_cusp[testIndex]);
        float approxLimit_J = J_intersect_source[testIndex] + slopes[testIndex] * approxLimit_M;

        // Store JMh values
        float approximate_JMh[3] = {approxLimit_J, approxLimit_M, test_JMh[testIndex][2]};

        // Convert to RGB
        float newLimitRGB[3] = JMh_to_RGB(approximate_JMh, limit_params);

        // Check if any values exceed the luminance limit. If so, we are outside of the top gamut shell.
        if (!outside_hull(newLimitRGB, luminance_limit))
            return false;
    }

    return true;
}

float[totalTableSize] make_upper_hull_gamma_table(float gamutCuspTable[totalTableSize][3],
                                                  ODTParams p)
{
    // Find upper hull gamma values for the gamut mapper.
    // Start by taking a h angle
    // Get the cusp J value for that angle
    // Find a J value halfway to the Jmax
    // Iterate through gamma values until the approximate max M is
    // negative through the actual boundary

    // positions between the cusp and Jmax we will check variables that get
    // set as we iterate through, once all are set to true we break the loop

    float upper_hull_gamma[totalTableSize];

    for (int i = baseIndex; i != baseIndex + tableSize; i = i + 1)
    {
        // Get cusp from cusp table at hue position
        float hue = gamutCuspTable[i][2];
        float JMcusp[2] = {gamutCuspTable[i][0], gamutCuspTable[i][1]};

        float test_JMh[test_count][3];
        float J_intersect_source[test_count];
        float slopes[test_count];
        float J_intersect_cusp[test_count];

        generate_gamma_test_data(JMcusp, hue, p.limit_J_max, p.mid_J, p.focus_dist,
                                 test_JMh, J_intersect_source, slopes, J_intersect_cusp);

        float search_range = gamma_search_step;
        float low = gamma_minimum;
        float high = low + search_range;
        bool outside = false;
        while (!(outside) && (high < gamma_maximum))
        {
            bool gammaFound = evaluate_gamma_fit(JMcusp,
                                                 test_JMh, J_intersect_source, slopes, J_intersect_cusp,
                                                 1. / high,
                                                 p.peakLuminance, p.limit_J_max, p.lower_hull_gamma_inv, p.limit_params);
            if (!gammaFound)
            {
                low = high;
                high = high + search_range;
            }
            else
            {
                outside = true;
            }
        }

        float testGamma = -1.0;
        while ((high - low) > gamma_accuracy)
        {
            testGamma = midpoint(high, low);
            bool gammaFound = evaluate_gamma_fit(JMcusp,
                                                 test_JMh, J_intersect_source, slopes, J_intersect_cusp,
                                                 1. / testGamma,
                                                 p.peakLuminance, p.limit_J_max, p.lower_hull_gamma_inv, p.limit_params);
            if (gammaFound)
            {
                high = testGamma;
            }
            else
            {
                low = testGamma;
            }
        }

        upper_hull_gamma[i] = 1. / high;
    }

    // Copy last populated entry to first empty spot
    upper_hull_gamma[0] = upper_hull_gamma[tableSize];

    // Copy first populated entry to last empty spot
    upper_hull_gamma[tableSize + baseIndex] = upper_hull_gamma[baseIndex];

    return upper_hull_gamma;
}

int[2] determine_hue_linearity_search_range(float hue_table[])
{
    // This function searches through the hues looking for the largest
    // deviations from a linear distribution. We can then use this to initialise
    // the binary search range to something smaller than the full one to reduce
    // the number of lookups per hue lookup from ~ceil(log2(table size)) to
    // ~ceil(log2(range)) during image rendering.

    const int lower_padding = 0;
    const int upper_padding = 1;

    int hue_linearity_search_range[2] = {lower_padding, upper_padding};

    for (int i = baseIndex; i != baseIndex + tableSize; i = i + 1)
    {
        const int pos = hue_position_in_uniform_table(hue_table[i], totalTableSize);
        const int delta = i - pos;
        hue_linearity_search_range[0] = min(hue_linearity_search_range[0], delta + lower_padding);
        hue_linearity_search_range[1] = max(hue_linearity_search_range[1], delta + upper_padding);
    }

    return hue_linearity_search_range;
}
