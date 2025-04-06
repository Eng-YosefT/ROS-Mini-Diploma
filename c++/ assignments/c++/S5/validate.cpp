/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : validate.cpp
 */
#include <iostream>
using namespace std;

class Date {
private:
    int day;
    int month;
    int year;

    bool isLeapYear(int year) {
        return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    }

    bool isValidDate(int d, int m, int y) {
        if (y < 1 || m < 1 || m > 12 || d < 1) {
            return false;
        }

        int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
        if (isLeapYear(y)) {
            daysInMonth[1] = 29; // February has 29 days in a leap year
        }

        return d <= daysInMonth[m - 1];
    }

public:
    void setDate(int d, int m, int y) {
        if (isValidDate(d, m, y)) {
            day = d;
            month = m;
            year = y;
        } else {
            cout << "Invalid date entered!" << endl;
            day = month = year = 0; // Reset to default invalid state
        }
    }

    int getDay() const {
        return day;
    }

    int getMonth() const {
        return month;
    }

    int getYear() const {
        return year;
    }

    void displayDate() const {
        if (day == 0 || month == 0 || year == 0) {
            cout << "No valid date set." << endl;
        } else {
            cout << "Date: " << day << "/" << month << "/" << year << endl;
        }
    }

    bool isValid() const {
        return isValidDate(day, month, year);
    }
};

int main() {
    Date date;
    int day, month, year;

    cout << "Input day: ";
    cin >> day;
    cout << "Input month: ";
    cin >> month;
    cout << "Input year: ";
    cin >> year;

    date.setDate(day, month, year);

    date.displayDate();

    if (date.isValid()) {
        cout << "The date is valid." << endl;
    } else {
        cout << "The date is invalid." << endl;
    }

    return 0;
}