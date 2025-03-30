#include <iostream>
using namespace std;
int main () {
    cout << "please enter the number of room you wont to reserve : ";
    int room_number;
    cin >> room_number;
    cout << "please enter the days you wont to stay : " ;
    int days;
    cin >> days;
    int total = room_number * days * 300;
    cout << "the cost is : " << total << endl;
    cout << "the tax is : " << total * 0.1 << endl;
    cout << "the total cost is: " << total * 1.10 << endl;
    return 0;
}