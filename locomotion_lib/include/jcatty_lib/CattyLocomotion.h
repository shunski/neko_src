// CattyLocomotion.h implemented by CattyLocomotion.cpp

#ifndef CATTYLOCOMOTION_H
#define CATTYLOCOMOTION_H

#include <vector>
#include <math.h>
#include <ros/ros.h>

#include <support_lib/Utilities.h> // add

class CattyLocomotion
{
    protected:
        Uint8Sequence sequence;
        unsigned int numOfSequence;
        ros::Duration period;
        int max_amplitude;
        int max_speed;
        bool valid;

        bool check_connectivity() const;
        bool check_periodicity() const;

        // Constructor (Initializer)
        CattyLocomotion(Uint8Sequence Sequence, ros::Duration Period, int MaxAmplitude, int MaxSpeed ) :
            phi( Sequence ),
            period( Period ),
            max_amplitude( MaxAmplitude ),
            max_speed( MaxSpeed ),
            valid(　true　)
        {
            if( !check() ) {
                valid = false;
                ROS_INFO("Failed to Construct CattyLocomotion.");
            }
        }


    public:
        bool check() const {
            if ( get_durationPerPlot < 0.01 ) return false;
            if ( !isPeriodic()) return false;
            if ( !isContinuous()) return false;
            if ( !isDifferentiable()) return false;
            return true;
        }

        std::vector<double> get_first_derivative() const;
        std::vector<double> get_second_derivative() const;
        void set_first_derivative( std::vector<double> & ) const;
        void set_second_derivative( std::vector<double> & ) const;
        double get_highest_speed() const ;
        double get_highest_torque() const ;
        ros::Duration get_durationPerPlot() const { return period / double(numOfSequence) };
        void set_MotionControllGoal( MotionControllGoal & );
};


namespace CattyLocomotion
{
    class ServoCmd : public CattyLocomotion
    {
        private:
            Uint8Sequence speedLimitSeqence;
            Uint8Sequence stretchLimitSequence;
            Uint8Sequence currentLimitSequence;
            Uint8Sequence temperatureLimitSequence;
        public:
            ServoCmd(Uint8Sequence Sequence, ros::Duration Period, Uint8Sequence LimitSpeedSeqence,
                Uint8Sequence LimitStretchSequence, Uint8Sequence LimitCurrentSequence) :
                CattyLocomotion(Sequence, Period),
                speedLimitSeqence(SpeedLimitSeqence),
                stretchLimitSeqence(StretchLimitSeqence),
                currentLimitSeqence(CurrentLimitSeqence),
                temperatureLimitSequence(TemperatureLimitSequence)
            {}
    };

    class BrushedMotorCmd : public CattyLocomotion
    {
        private:
            Uint8Sequence currentLimitSequence;
        public:
            BrushedMotorCmd(Uint8Sequence Sequence, ros::Duration Period, int MaxSpeed, int MaxAmplitude, Uint8Sequence CurrentLimitSeqence) :
                CattyLocomotion( Sequence, Period, MaxSpeed, MaxAmplitude ),
                currentLimitSeqence( CurrentLimitSeqence )
            {}


    };

    class PartCmd
    {
        private:
            std::vector<ServoCmd> servoCommands;
            std::vector<BrushedMotorCmd> brushedMotorCommands;
            std::vector<BrushlessMotorCmd> brushlessMotorCommands;
        public:
            PartCmd(std::vector<ServoCmd> ServoCommands, std::vector<DCmotorCmd> BrushedMotorCommands):
                servoCommands(ServoCommands),
                brushedMotorCommands(BrushedMotorCommands),
                brushlessMotorCommands(BrushlessMotorCommands)
            {}
    };

    Uint8Sequence createConstantSequence( unsigned char a, unsigned int size ) {
        return Uint8Sequence(size, a);
    }

    void createSampleCmd
}
