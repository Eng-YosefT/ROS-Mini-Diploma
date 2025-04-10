#include <iostream>
#include <memory>
using namespace std;

class Person; // Forward declaration

class Dog {
public:
    string name;
    weak_ptr<Person> owner;  // weak_ptr to avoid circular reference

    Dog(const string& n) : name(n) {
        cout << "Dog constructed: " << name << endl;
    }

    ~Dog() {
        cout << "Dog destructed: " << name << endl;
    }

    void showOwner() {
        if (auto sharedOwner = owner.lock()) { // Check if Person still exists
            cout << "Dog " << name << "'s owner is: " << sharedOwner->name << endl;
        } else {
            cout << "Dog " << name << " has no owner." << endl;
        }
    }
};

class Person {
public:
    string name;
    shared_ptr<Dog> pet;

    Person(const string& n) : name(n) {
        cout << "Person constructed: " << name << endl;
    }

    ~Person() {
        cout << "Person destructed: " << name << endl;
    }

    void showPet() {
        if (pet) {
            cout << name << "'s dog is: " << pet->name << endl;
        } else {
            cout << name << " has no dog." << endl;
        }
    }
};

int main() {
    shared_ptr<Person> p = make_shared<Person>("Alice");
    shared_ptr<Dog> d = make_shared<Dog>("Buddy");

    p->pet = d;          // Person owns a dog
    d->owner = p;        // Dog knows its owner (weak_ptr)

    p->showPet();
    d->showOwner();

    // Reset shared pointers to explicitly destroy objects
    p.reset();
    d.reset();

    // After main ends, both Person and Dog are destroyed properly
    return 0;
}
