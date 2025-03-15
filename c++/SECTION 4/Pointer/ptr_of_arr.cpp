#include <iostream> //by youssef taha 15/3/2025
using namespace std;
int main() {
    int arr[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    int *ptr {arr} ;// pointer to the first element of the array
    for (int i = 0; i < 10; i++) {
        cout << "element " << i+1  << " = " << ptr [i] << endl;
    }
    return 0;
}