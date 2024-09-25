#ifndef __PARAMETERS__
#define __PARAMETERS__

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class Parameters{
    public:
        Parameters() : Parameters(2.0, 6.0, 0.115, 0.115, 0.001){};

        Parameters(double v_max, double a_max, double veh_width, 
                   double veh_height, double margin){
            v_max_ = v_max;
            a_max_ = a_max;
            veh_width_ = veh_width;
            veh_height_ = veh_height;
            margin_ = margin;
        };

        // basic getters
        double GetVmax() const { return v_max_;};
        double GetAmax() const { return a_max_;};
        double GetVehWidth() const { return veh_width_;};
        double GetVehHeight() const { return veh_height_;};
        double GetMargin() const { return margin_;};
        double GetWidthOffset() const { return veh_width_/2 + margin_;};
        double GetHeightOffset() const { return veh_height_/2 + margin_;};

        // basic setters
        void SetVmax(double v_max){ v_max_ = v_max;};
        void SetAmax(double a_max){ a_max_ = a_max;};
        void SetVehHeight(double veh_height){ veh_height_ = veh_height;};
        void SetVehWidth(double veh_width){ veh_width_ = veh_width;};
        void SetMargin(double margin){ margin_ = margin;};

        json ToJson() const {
            return json{{"v_max", v_max_}, {"a_max", a_max_}, 
                        {"veh_width", veh_width_}, {"veh_height", veh_height_},
                        {"margin", margin_}};
        };

    private:
        double v_max_;       // Maximum (horizontal or vertical) velocity
        double a_max_;       // Maximum (horizontal or vertical) acceleration
        
        double veh_width_;   // Width of the vehicle (in 2D top-down view)
        double veh_height_;  // Height of the vehicle (in 2D top-down view)
        double margin_;      // Margin between vehicle and corridor bounds
};


#endif