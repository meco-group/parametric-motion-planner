#ifndef __PARAMETRIZATION__
#define __PARAMETRIZATION__

class Parametrization{
    public:
        Parametrization() : max_nb_corridors_(20){};
        Parametrization(int max_nb_corridors) : max_nb_corridors_(max_nb_corridors){};

    private:
        const int max_nb_corridors_;   
};

#endif