#ifndef __PARAMETERS__
#define __PARAMETERS__

class Parameters{
    public:
        Parameters(){
            // assign default values to class attributes
            v_max_ = 2.0;
            a_max_ = 6.0;
            veh_width_ = 0.115;
            veh_height_ = 0.115;
            margin_ = 0.001;
        };

        Parameters(double v_max, double a_max, double veh_width, 
                   double veh_height, double margin){
            v_max_ = v_max;
            a_max_ = a_max;
            veh_width_ = veh_width;
            veh_height_ = veh_height;
            margin_ = margin;
        };

        double GetVmax(){ return v_max_;};
        double GetAmax(){ return a_max_;};
        double GetVehWidth(){ return veh_width_;};
        double GetVehHeight(){ return veh_height_;};
        double GetMargin(){ return margin_;};

        void SetVmax(double v_max){ v_max_ = v_max;};
        void SetAmax(double a_max){ a_max_ = a_max;};
        void SetVehHeight(double veh_height){ veh_height_ = veh_height;};
        void SetVehWidth(double veh_width){ veh_width_ = veh_width;};
        void SetMargin(double margin){ margin_ = margin;};

    private:
        double v_max_;       // Maximum (horizontal or vertical) velocity
        double a_max_;       // Maximum (horizontal or vertical) acceleration
        
        double veh_width_;   // Width of the vehicle (in 2D top-down view)
        double veh_height_;  // Height of the vehicle (in 2D top-down view)
        double margin_;      // Margin between vehicle and corridor bounds

};


#endif