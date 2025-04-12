#include <iostream>
using namespace std;

// // تعريف كلاس جديد اسمه Student
// // الكلاس هو قالب (template) بنستخدمه لإنشاء كائنات (Objects) لها نفس الخصائص والدوال
class Student {
private:
    // // الخصائص (Attributes) أو المتغيرات الخاصة بالكلاس - private يعني مش هتكون متاحة من بره الكلاس مباشرة
    string name;  // اسم الطالب
    int age;      // سن الطالب
    float grade;  // درجة الطالب

public:
    // // Constructor (دالة خاصة بتشتغل أول ما الكائن يتكون)
    // // وظيفتها تهيّء القيم الابتدائية للخصائص
    Student(string n, int a, float g) {
        name = n;
        age = a;
        grade = g;
    }

    // // Setter methods - بتستخدم لتعديل الخصائص من بره الكلاس
    void setName(string n) {
        name = n;
    }

    void setAge(int a) {
        age = a;
    }

    void setGrade(float g) {
        grade = g;
    }

    // // Getter methods - بتستخدم لقراءة القيم من داخل الكلاس
    string getName() {
        return name;
    }

    int getAge() {
        return age;
    }

    float getGrade() {
        return grade;
    }

    // // دالة لعرض المعلومات الخاصة بالكائن
    void displayInfo() {
        cout << "Name: " << name << "\nAge: " << age << "\nGrade: " << grade << endl;
    }
};

// // دالة main هي نقطة البداية لأي برنامج C++
int main() {
    // // إنشاء كائن (Object) من الكلاس Student واسمه s1
    Student s1("Youssef", 21, 89.5);

    // // عرض بيانات الطالب باستخدام دالة displayInfo
    s1.displayInfo();

    // // تعديل الدرجة باستخدام setter
    s1.setGrade(95.0);

    // // عرض البيانات بعد التعديل
    cout << "\nAfter update:\n";
    s1.displayInfo();

    return 0;
}
