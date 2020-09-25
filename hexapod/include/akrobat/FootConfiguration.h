#ifndef __FOOT_POSITION__
#define __FOOT_POSITION__

#include <iostream>
#include <array>
#include <vector>
#include <initializer_list>
#include <functional>
#include <tf/transform_datatypes.h>

class FootConfiguration {
    public:
        static constexpr std::size_t FOOT_SIZE = 6; 

        enum FootBit
        {
            FOOT_EL_1 = (1 << 0),
            FOOT_EL_2 = (1 << 1),
            FOOT_EL_3 = (1 << 2),
            FOOT_EL_4 = (1 << 3),
            FOOT_EL_5 = (1 << 4),
            FOOT_EL_6 = (1 << 5)
        };

        static signed char FOOT_BIT[6];

        static std::vector<signed char> validStates;
        static std::vector<std::vector<signed char>> transitions;

        signed char state = FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_5 | FOOT_EL_6;

        static void init();

        struct Foot {
            Foot() {}
            Foot(const tf::Vector3& position, const tf::Vector3& initPosition, const tf::Vector3& delta)
                : position(position), initPosition(initPosition), delta(delta)
            {}

            tf::Vector3 position;
            tf::Vector3 initPosition;
            tf::Vector3 delta;
        };
        
        using Position = std::array<Foot, FOOT_SIZE>;

        FootConfiguration() {
            
        }
        FootConfiguration(const signed char state, const Position& position);
        FootConfiguration deepCopy();

        Position get() const;

        Position currentPosition; 
};

std::ostream& operator<<(std::ostream& os, const FootConfiguration& position);

#endif //__FOOT_POSITION__
