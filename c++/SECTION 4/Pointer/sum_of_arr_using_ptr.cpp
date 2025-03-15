#include <iostream>
using namespace std;
int main() {
    int *arr = new int[5];
    for (int i = 0; i < 5; ++i) {
        cout << "Enter num [" << i+1 << "] = ";
        cin >> *(arr+i);
    }
    for (int i = 0; i < 5; ++i) {
        cout << "arr[" << i << "] = " << arr[i] << endl;
    }
    // prin sum of the array
    int sum = 0;
    for (int i = 0; i < 5; ++i) {
        sum += *(arr+i);
    }
    cout << "sum of the array = " << sum << endl;
    delete[] arr;
    return 0;
}