//
// Created by lixin04 on 20年1月14日.
//

#ifndef LVO_PARAM_HPP
#define LVO_PARAM_HPP

namespace LVO{
    enum LineExtractType{ LSD,  };
// 单目线提取参数
    class MonoParam{
    public:
        MonoParam(){}


        bool equalize = true;
        LineExtractType line_extract_type = LineExtractType::LSD;

        int MaxNumLineFeatures = 300;
    };

    class StereoMatchParam{
    public:
        StereoMatchParam(){}



        float match_dist_thread = 30;

//    private:

    };
}

#endif //LVO_PARAM_HPP
