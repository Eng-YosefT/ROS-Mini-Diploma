/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : 5-dog_constructor.cpp
*/
 #include <iostream>
 #include <string>
 
 class Dog {
 private:
     std::string name;
     int age = 0;
 
 public:
     // Setter for name
     void setName(std::string dogName) {
         name = dogName;
     }
 
     // Getter for name
     std::string getName() const {
         return name;
     }
 
     // Setter for age
     void setAge(int dogAge) {
         if (dogAge >= 0) { 
             age = dogAge;
         } else {
             std::cerr << "Age cannot be negative!" << std::endl;
         }
     }
 
    // Constructor to initialize only name
    Dog(std::string dogName) {
        name = dogName;
        age = 0; // Default age
    }

    // Constructor to initialize only age
    Dog(int dogAge) {
        if (dogAge >= 0) {
            age = dogAge;
        } else { 
            std::cerr << "Age cannot be negative! Setting age to 0." << std::endl;
            age = 0;
        }
        name = "Unknown"; // Default name
    }
    // Default constructor
    Dog() {
       cout << "Done make a new dog" << endl;  
    }
    // Functions to set both name and age
     // Getter for age
     int getAge() const {
         return age;
     }
    // Constructor to initialize name and age
    Dog(std::string dogName, int dogAge) {
        name = dogName;
        if (dogAge >= 0) {
            age = dogAge;
        } else {
            std::cerr << "Age cannot be negative! Setting age to 0." << std::endl;
            age = 0;
        }
    }
 };
 
 int main() {
    Dog myDog("Buddy", 3);
 

     std::cout << "Dog's Name: " << myDog.getName() << std::endl;
     std::cout << "Dog's Age: " << myDog.getAge() << " years old" << std::endl;
 
     return 0;
 }