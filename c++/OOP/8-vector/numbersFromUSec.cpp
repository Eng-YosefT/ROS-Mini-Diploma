/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-8
 * File       : numbersFromUSec.cpp
 * Description: 
 */
#include <iostream>
#include <vector>
using namespace std;

int main() {
    vector<int> numbers;
    int num;

    cout << "Enter numbers (enter -1 to stop):" << endl;
    while (true) {
        cin >> num;
        if (num == -1)
            break;
        numbers.push_back(num);
    }

    cout << "\nNumbers in reverse order:" << endl;
    for (int i = numbers.size() - 1; i >= 0; --i) {
        cout << numbers[i] << " ";
    }
    cout << endl;

    return 0;
}
