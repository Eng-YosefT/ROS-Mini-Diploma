#include<iostream> 
using namespace std; 

int num1 = 0; 
int num2 = 0; 

void swap(int &num1, int &num2); 

int main() 
{ 
    cout << "enter num1: ";
    cin >> num1; 
    cout << "enter num2: "; 
    cin >> num2; 
    cout << "number before swap: " << num1 << " " << num2 << endl; 
    swap(num1, num2); 
    cout << "number after swap: " << num1 << ", " << num2 << endl; 
    return 0;
} 

void swap(int &num1, int &num2)  
{ 
    int temp = num1; 
    num1 = num2; 
    num2 = temp; 
}
