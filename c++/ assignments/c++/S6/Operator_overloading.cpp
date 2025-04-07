/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : Operator_overloading.cpp
 */
#include <iostream>
using namespace std;
class Integer {
private:
    int value; 

public:
    // Constructor 
    Integer(int v = 0) {
        value = v;
    }

    // Overload للمشغل +
    Integer operator+(int other) {
        return Integer(value + other);
    }

    // Overload للمشغل -
    Integer operator-(int other) {
        return Integer(value - other);
    }

    // Overload للمشغل *
    Integer operator*(int other) {
        return Integer(value * other);
    }

    // Overload للـ postfix ++
    Integer operator++(int) {
        Integer temp = *this;
        value++;
        return temp;
    }

    // Overload للـ postfix --
    Integer operator--(int) {
        Integer temp = *this;
        value--;
        return temp;
    }

    // Overload للمقارنة >
    bool operator>(int other) {
        return value > other;
    }

    // Overload للمقارنة !=
    bool operator!=(int other) {
        return value != other;
    }

    // Getter (يرجع قيمة value)
    int getValue() {
        return value;
    }
};

int main() {
    Integer num(2); 

    cout << "num = " << num.getValue() << endl;
    cout << "num + 52 = " << (num + 52).getValue() << endl;
    cout << "num - 1 = " << (num - 1).getValue() << endl;
    cout << "num * 3 = " << (num * 3).getValue() << endl;
    cout << "num++ = " << (num++).getValue() << endl; 
    cout << "num-- = " << (num--).getValue() << endl; 
    cout << "num > 0 is " << (num > 0 ? "true" : "false") << endl;
    cout << "num != 3 is " << (num != 3 ? "true" : "false") << endl;

    return 0;
}
