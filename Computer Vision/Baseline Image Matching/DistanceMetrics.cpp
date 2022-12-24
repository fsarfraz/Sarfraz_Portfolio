#include "Header.h"

int SSD(char *img_name, vector<float> &t_features, vector<float> &d_features, vector<tuple<string,float>> &result) {
    float sum = 0.0; 
   
    for (int i = 0; i < t_features.size(); i++) {
        float diff = pow(t_features[i] - d_features[i], 2);
        sum = sum + diff;
    }
    sum = sum / 243;
    
    tuple<string, float> vec(img_name,sum);
    result.push_back(vec);
  
    return (0);
}

int hg_intersection(char* img_name, vector<float> &t_features, vector<float> &d_features, vector<tuple<string, float>> &result) {
    float hist1 = 0.0;
    float hist2 = 0.0;
    float sum = 0.0;

    for (int i = 0; i < t_features.size(); i++) {
        sum += min(t_features[i], d_features[i]);
        hist1 += t_features[i];
        hist2 += d_features[i];
    }
    float intersection = 1 - (sum / hist1);
    tuple<string, float> vec(img_name, intersection);
    
    result.push_back(vec);
    return (0);
}

int TextureColor_metric(char* img_name, vector<float> &T_colorhist, vector<float> &T_texthist,
                                        vector<float> &D_colorhist, vector<float> &D_texthist,
                                        vector<tuple<string, float>> &result) 
{
    float sum = 0.0;
    float sum2 = 0.0;
    float histT_color = 0.0;
    float histT_texture= 0.0;
    float histD_color = 0.0;
    float histD_texture = 0.0;

    //intesection for color histograms
    for (int i = 0; i < T_colorhist.size(); i++) {
        sum += min(T_colorhist[i], D_colorhist[i]);
        histT_color += T_colorhist[i];
        histD_color += D_colorhist[i];
    }
    float color_intersection = 1 - (sum / histT_color);

    // intersection for texture histogram
    for (int i = 0; i < T_texthist.size(); i++) {
        sum2 += min(T_texthist[i], D_texthist[i]);
        histT_texture += T_texthist[i];
        histD_texture += D_texthist[i];
    }
    float texture_intersection = 1 - (sum2 / histT_texture);

    //Weighting histograms evenly
    float w_dist = 0.5 * color_intersection + 0.5 * texture_intersection;

    tuple<string, float> vec(img_name, w_dist);
    result.push_back(vec);
    return (0);
}

int TextureColorHSV_metric(char* img_name, vector<float>& T_colorhist, vector<float>& T_texthist, vector<float>& T_hsvhist,
                                           vector<float>& D_colorhist, vector<float>& D_texthist, vector<float>& D_hsvhist,
                                            vector<tuple<string, float>>& result)
{
    float sum = 0.0;
    float sum2 = 0.0;
    float sum3 = 0.0;
    float histT_color = 0.0;
    float histT_texture = 0.0;
    float histT_hsv = 0.0;
    float histD_color = 0.0;
    float histD_texture = 0.0;
    float histD_hsv = 0.0;

    //intesection for color histograms
    for (int i = 0; i < T_colorhist.size(); i++) {
        sum += min(T_colorhist[i], D_colorhist[i]);
        histT_color += T_colorhist[i];
        histD_color += D_colorhist[i];
    }
    float color_intersection = 1 - (sum / histT_color);

    // intersection for texture histogram
    for (int i = 0; i < T_texthist.size(); i++) {
        sum2 += min(T_texthist[i], D_texthist[i]);
        histT_texture += T_texthist[i];
        histD_texture += D_texthist[i];
    }
    float texture_intersection = 1 - (sum2 / histT_texture);

    for (int i = 0; i < T_hsvhist.size(); i++) {
        sum3 += min(T_hsvhist[i], D_hsvhist[i]);
        histT_hsv += T_hsvhist[i];
        histD_hsv += D_hsvhist[i];
    }
    float hsv_intersection = 1 - (sum3 / histT_hsv);

    //Weighting histograms evenly
    float w_dist = 0.4 * color_intersection + 0.2 * texture_intersection + 0.4 * hsv_intersection;

    tuple<string, float> vec(img_name, w_dist);
    result.push_back(vec);
    return (0);
}