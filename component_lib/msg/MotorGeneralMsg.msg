# BrushedMotorMsg
uint8 part_id
uint8 id
uint8 current_limit
uint8 pwm
uint8 current
uint16 rpm

# See definitions in BrushedMotor.h



// inheritance
#include <iostream>
#include <vector>
using namespace std;

class Parent {
    public:
        // virtual void hello()=0;
        virtual void hello() { cout << "hello from the Parent" << endl; }
};

class Child1 : public Parent {
    public:
        void hello() override { cout << "hello from the Child1" << endl; }
};

class Child2 : public Parent {
    public:
        void hello() override { cout << "hello from the Child2" << endl; }
};


int main()
{
    vector<Parent> group = {Child1(), Child2()};
    group.front().hello();

    return 0;
}
