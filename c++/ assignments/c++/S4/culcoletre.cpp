#include <iostream>
using namespace std;
int main() {
    char op;
    float num1, num2, result;
    float *ptr1 = &num1, *ptr2 = &num2, *ptrResult = &result;

    cout << "Enter operator (+, -, *, /): ";
    cin >> op;
    cout << "Enter two operands: ";
    cin >> *ptr1 >> *ptr2;

    switch(op) {
        case '+':
            *ptrResult = *ptr1 + *ptr2;
            break;
        case '-':
            *ptrResult = *ptr1 - *ptr2;
            break;
        case '*':
            *ptrResult = *ptr1 * *ptr2;
            break;
        case '/':
            if (*ptr2 != 0)
                *ptrResult = *ptr1 / *ptr2;
            else {
                cout << "Error! Division by zero.";
                return 1;
            }
            break;
        default:
            cout << "Error! Operator is not correct";
            return 1;
    }

    cout << "Result: " << *ptrResult << endl;
    return 0;
}