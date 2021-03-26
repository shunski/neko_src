// CattyLocomotion.h implemented by CattyLocomotion.cpp

#ifndef CATTYLOCOMOTION_H
#define CATTYLOCOMOTION_H

#include <array>
#include <vector>
#include <math.h>
#include <ros/ros.h>

#define numOfTerms 361

typedef boost::array<unsigned, numOfTerms> Uint8Sequence;

class CattyLocomotion
{
    protected:
        // (max_amplitude)*sin(2*pi*phi+initial_phase)
        Uint8Sequence phi;                        // x is in (0,numOfTerms-1); y is in (0,100);
        ros::Duration period;                     // domain of t
        int max_amplitude;
        int initial_phase;                        // 0 ~ numOfTerms
        int max_speed;                            // [Hz]
        bool valid;

        bool check_monotonicity() const ;
        bool check_connectivity() const;
        bool check_max_speed() const ;

        // Constructor (Initializer)
        CattyLocomotion(Uint8Sequence Sequence, double Period) :
            phi(Sequence),
            period(Period),
            valid(true)
        {
            if(!check()) {
                valid = false;
                ROS_INFO("Failed to Construct CattyLocomotion.");
            }
        }


    public:
        bool check() const {
            if (check_monotonicity()) return false;
            if (check_connectivity()) return false;
            return true;
        }

        array<double, numOfTerms> get_first_derivative() const ;
        array<double, numOfTerms> get_second_derivative()const ;
        double get_highest_speed() const ;
        double get_highest_torque() const ;
};


namespace CattyLocomotion
{
    class ServoCmd : public CattyLocomotion
    {
        private:
            Uint8Sequence limitSpeedSeqence;
            Uint8Sequence limitStretchSequence;
            Uint8Sequence limitCurrentSequence;
        public:
            ServoCmd(Uint8Sequence Sequence, double Period, Uint8Sequence LimitSpeedSeqence,
                Uint8Sequence LimitStretchSequence, Uint8Sequence LimitCurrentSequence) :
            CattyLocomotion(Sequence, Period),
            limitSpeedSeqence(LimitSpeedSeqence),
            limitStretchSeqence(LimitStretchSeqence),
            limitCurrentSeqence(LimitCurrentSeqence)
            {}
    };

    class DCmotorCmd : public CattyLocomotion
    {
        private:
            Uint8Sequence currentLimitSequence;
            Uint8Sequence stretchLimitSequence;
        public:
            DCmotorCMD() :
            CattyLocomotion(Sequence, Duration):
            {}
    };

    class PartCmd
    {
        private:
            std::vector<ServoCmd> servoCommands;
            std::vector<DCmotorCmd> dcmotorCommands;
        public:
            PartCmd(std::vector<ServoCmd> ServoCommands, std::vector<DCmotorCmd> DCmotorCommands):
            servoCommands(ServoCommands),
            dcmotorCommands(DCmotorCommands)
            {}
    };

    class BodyCmd
    {
        private:
            PartCmd headCmd;
            PartCmd chestCmd;
            PartCmd waistCmd;
            PartCmd rfCmd;
            PartCmd lfCmd;
            PartCmd rhCmd;
            PartCmd lfCmd;
        public:
            BodyCmd(PartCmd HeadCmd, PartCmd ChestCmd, PartCmd WaistCmd, PartCmd RFCmd, PartCmd LFCmd, PartCmd RHCmd, PartCmd LFCmd):
            headCmd(HeadCmd),
            chestCmd(ChestCmd),
            waistCmd(WaistCmd),
            rfCmd(RFCmd),
            lfCmd(LFCmd),
            rhCmd(RHCmd),
            lfCmd(LFCmd)
            {}
    };
}
