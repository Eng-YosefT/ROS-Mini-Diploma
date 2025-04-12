/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-06
 * File       : bank.cpp
 * Description: This file contains the implementation of a simple banking system using inheritance in C++.
 *              It defines a base class 'BankAccount' and two derived classes 'SavingsAccount' and 'CheckingAccount'.
 *              The 'BankAccount' class contains basic account information and methods for deposit and withdrawal.
 *              The 'SavingsAccount' class adds functionality for interest calculation,
 *              while the 'CheckingAccount' class includes transaction fees.
 *              The program demonstrates how to create and manage different types of bank accounts.
 *              The code is written in C++ and uses basic OOP principles.
 */
#include <iostream>
using namespace std;

// Base class
class BankAccount {
protected:
    string accountNumber;
    double balance;

public:
    BankAccount(string accNum, double bal) {
        accountNumber = accNum;
        balance = bal;
    }

    void deposit(double amount) {
        balance += amount;
    }

    void withdraw(double amount) {
        if (amount <= balance)
            balance -= amount;
        else
            cout << "Insufficient balance!" << endl;
    }

    virtual void displayInfo() {
        cout << "Account Number: " << accountNumber << endl;
        cout << "Balance: " << balance << endl;
    }
};

// Derived class: SavingsAccount
class SavingsAccount : public BankAccount {
private:
    double interestRate;

public:
    SavingsAccount(string accNum, double bal, double rate)
        : BankAccount(accNum, bal), interestRate(rate) {}

    void calculateInterest() {
        double interest = balance * interestRate;
        balance += interest;
    }
};

// Derived class: CheckingAccount
class CheckingAccount : public BankAccount {
private:
    double transactionFee;

public:
    CheckingAccount(string accNum, double bal, double fee)
        : BankAccount(accNum, bal), transactionFee(fee) {}

    void deductTransactionFee() {
        if (transactionFee <= balance)
            balance -= transactionFee;
        else
            cout << "Not enough balance to deduct transaction fee!" << endl;
    }

    void displayInfo() override {
        cout << "Account Number: " << accountNumber << endl;
        cout << "Balance: " << balance << endl;
        cout << "Transaction Fee: " << transactionFee << endl;
    }
};

// Main function
int main() {
    // Create SavingsAccount object
    SavingsAccount savings("SA123", 1000.0, 0.05);
    savings.deposit(200.0);
    savings.calculateInterest();
    cout << "Savings Account Info:" << endl;
    savings.displayInfo();

    cout << endl;

    // Create CheckingAccount object
    CheckingAccount checking("CA456", 1500.0, 15.0);
    checking.withdraw(100.0);
    checking.deductTransactionFee();
    cout << "Checking Account Info:" << endl;
    checking.displayInfo();

    return 0;
}
