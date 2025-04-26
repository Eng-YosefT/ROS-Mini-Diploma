/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-20
 * File       : Library_Project.cpp
 * Description: 
 */
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <ctime>
#include <limits> 

using namespace std;

// Forward declarations
class Book;
class BorrowRecord;
class User;
class Librarian;
class Member;

// --- Helper Function for Input Validation ---
int getValidatedIntInput(const string& prompt) {
    int value;
    cout << prompt;
    while (!(cin >> value)) {
        cout << "Invalid input. Please enter a number: ";
        cin.clear(); // Clear error flags
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard invalid input
    }
    cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard the trailing newline
    return value;
}

// --- Date utility class (Improved) ---
class Date {
private:
    int day, month, year;

    // Helper to check for leap year
    bool isLeap(int y) const {
        return (((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0));
    }

    // Helper to get days in a month
    int daysInMonth(int m, int y) const {
        if (m == 2) {
            return isLeap(y) ? 29 : 28;
        } else if (m == 4 || m == 6 || m == 9 || m == 11) {
            return 30;
        } else {
            return 31;
        }
    }

public:
    Date() {
        time_t now = time(0);
        tm* ltm = localtime(&now);
        day = ltm->tm_mday;
        month = 1 + ltm->tm_mon;
        year = 1900 + ltm->tm_year;
    }

    Date(int d, int m, int y) : day(d), month(m), year(y) {}

    // More robust addDays implementation
    Date addDays(int days) const {
        Date result = *this;
        for (int i = 0; i < days; ++i) {
            int maxDays = daysInMonth(result.month, result.year);
            if (result.day < maxDays) {
                result.day++;
            } else {
                result.day = 1;
                if (result.month < 12) {
                    result.month++;
                } else {
                    result.month = 1;
                    result.year++;
                }
            }
        }
        return result;
    }

    bool isOverdue() const {
        Date today;
        if (year < today.year) return true;
        if (year == today.year && month < today.month) return true;
        if (year == today.year && month == today.month && day < today.day) return true;
        return false;
    }

    string toString() const {
        return to_string(day) + "/" +
               to_string(month) + "/" +
               to_string(year);
    }
};

// --- Book class ---
class Book {
private:
    string title;
    string author;
    string isbn;
    bool isAvailable;

public:
    Book(const string& t, const string& a, const string& i)
        : title(t), author(a), isbn(i), isAvailable(true) {}

    // Getters (const correct)
    string getTitle() const { return title; }
    string getAuthor() const { return author; }
    string getISBN() const { return isbn; }
    bool getAvailability() const { return isAvailable; }

    // Setters
    void setAvailability(bool status) { isAvailable = status; }

    // Display book information (const correct)
    void displayInfo() const {
        cout << "  Title: " << title << "\n"
             << "  Author: " << author << "\n"
             << "  ISBN: " << isbn << "\n"
             << "  Status: " << (isAvailable ? "Available" : "Borrowed");
    }
};

// --- BorrowRecord class ---
class BorrowRecord {
private:
    shared_ptr<Book> book;
    int borrowerId;
    Date borrowDate; // Keep track of when it was borrowed
    Date dueDate;

public:
    BorrowRecord(shared_ptr<Book> b, int id)
        : book(b), borrowerId(id), borrowDate(Date()) {
        // Set due date to 14 days from borrow date
        dueDate = borrowDate.addDays(14);
    }

    // Getters (const correct)
    shared_ptr<Book> getBook() const { return book; }
    int getBorrowerId() const { return borrowerId; }
    Date getDueDate() const { return dueDate; }

    bool isOverdue() const {
        return dueDate.isOverdue();
    }

    // Display record information (const correct)
    void displayInfo() const {
        cout << "  Book Title: " << book->getTitle() << "\n"
             << "  ISBN: " << book->getISBN() << "\n"
             << "  Borrower ID: " << borrowerId << "\n"
             << "  Due Date: " << dueDate.toString() << "\n"
             << "  Status: " << (isOverdue() ? "OVERDUE" : "On time");
    }
};

// --- Abstract User base class ---
class User {
protected:
    string name;
    int id;

public:
    User(const string& n, int i) : name(n), id(i) {}
    virtual ~User() = default; // Virtual destructor

    // Getters (const correct)
    string getName() const { return name; }
    int getId() const { return id; }

    // Pure virtual function for menu display
    virtual void displayMenu() const = 0;

    // Virtual function to process user choice (takes non-const refs as it modifies library state)
    virtual void processChoice(int choice, vector<shared_ptr<Book>>& books,
                               vector<shared_ptr<BorrowRecord>>& allRecords) = 0;
};


// --- Helper function for searching books (Refactored) ---
void searchBooksHelper(const vector<shared_ptr<Book>>& books) {
    if (books.empty()) {
        cout << "No books in the library.\n";
        return;
    }

    int searchType = getValidatedIntInput("Search by:\n1. Title\n2. Author\nEnter your choice: ");

    string searchTerm;
    cout << "Enter search term: ";
    getline(cin, searchTerm); // Read the actual term

    bool found = false;
    cout << "\n==== Search Results ====\n";

    for (const auto& book : books) {
        bool match = false;
        string lowerSearchTerm = searchTerm; // Basic case-insensitive prep (can be improved)
        transform(lowerSearchTerm.begin(), lowerSearchTerm.end(), lowerSearchTerm.begin(), ::tolower);

        string lowerTitle = book->getTitle();
        transform(lowerTitle.begin(), lowerTitle.end(), lowerTitle.begin(), ::tolower);

        string lowerAuthor = book->getAuthor();
        transform(lowerAuthor.begin(), lowerAuthor.end(), lowerAuthor.begin(), ::tolower);

        if (searchType == 1) { // Title search
            match = lowerTitle.find(lowerSearchTerm) != string::npos;
        } else if (searchType == 2) { // Author search
            match = lowerAuthor.find(lowerSearchTerm) != string::npos;
        } else {
             cout << "Invalid search type selected.\n";
             return; // Exit if invalid type chosen
        }

        if (match) {
            cout << "\n"; // Newline before book info
            book->displayInfo();
            cout << "\n"; // Newline after book info
            found = true;
        }
    }

    if (!found) {
        cout << "No matching books found.\n";
    }
}


// --- Librarian class derived from User ---
class Librarian : public User {
public:
    Librarian(const string& name, int id) : User(name, id) {}

    void displayMenu() const override { // Marked const
        cout << "\n==== Librarian Menu (" << name << ") ====\n";
        cout << "1. Add a new book\n";
        cout << "2. Remove a book\n";
        cout << "3. View all books\n";
        cout << "4. View all borrowing records\n";
        cout << "5. Search books\n";
        cout << "6. View overdue books\n";
        cout << "7. Logout\n";
        // Prompt is handled by getValidatedIntInput
    }

    void processChoice(int choice, vector<shared_ptr<Book>>& books,
                       vector<shared_ptr<BorrowRecord>>& allRecords) override {
        switch (choice) {
            case 1: addBook(books); break;
            case 2: removeBook(books, allRecords); break;
            case 3: viewAllBooks(books); break;
            case 4: viewAllBorrowRecords(allRecords); break;
            case 5: searchBooks(books); break; // Calls refactored helper
            case 6: viewOverdueBooks(allRecords); break;
            case 7: cout << "Logging out...\n"; break; // Logout handled in main loop
            default: cout << "Invalid choice. Please try again.\n"; break;
        }
    }

private:
    void addBook(vector<shared_ptr<Book>>& books) {
        string title, author, isbn;

        // cin.ignore() is handled by getValidatedIntInput or subsequent getlines
        cout << "Enter book title: ";
        getline(cin, title);

        cout << "Enter author name: ";
        getline(cin, author);

        cout << "Enter ISBN: ";
        getline(cin, isbn);

        // Check if ISBN already exists
        auto it = find_if(books.begin(), books.end(),
                          [&isbn](const shared_ptr<Book>& book) {
                              return book->getISBN() == isbn;
                          });

        if (it != books.end()) {
            cout << "Error: A book with this ISBN already exists.\n";
            return;
        }

        books.push_back(make_shared<Book>(title, author, isbn));
        cout << "Book added successfully.\n";
    }

    void removeBook(vector<shared_ptr<Book>>& books,
                   const vector<shared_ptr<BorrowRecord>>& records) { // records can be const
        string isbn;
        cout << "Enter ISBN of book to remove: ";
        getline(cin, isbn);

        auto it = find_if(books.begin(), books.end(),
                          [&isbn](const shared_ptr<Book>& book) {
                              return book->getISBN() == isbn;
                          });

        if (it != books.end()) {
            // Check if the book is currently borrowed by searching *all* records
            bool isBorrowed = false;
            for (const auto& record : records) {
                // Important: Check if the record points to the *same book object*
                if (record->getBook() == *it) {
                    // Also check if this record is still active (i.e., book is marked unavailable)
                    // This check assumes a returned book's record might linger in allRecords
                    // A better approach might be to remove records from allRecords upon return,
                    // or add a 'returnedDate' field to BorrowRecord.
                    // For now, we check the book's current availability flag.
                    if (!(*it)->getAvailability()) {
                         isBorrowed = true;
                         break;
                    }
                }
            }

            if (isBorrowed) {
                cout << "Error: Cannot remove this book as it is currently borrowed.\n";
            } else {
                books.erase(it);
                cout << "Book removed successfully.\n";
            }
        } else {
            cout << "Book with ISBN " << isbn << " not found.\n";
        }
    }

    // View methods are const
    void viewAllBooks(const vector<shared_ptr<Book>>& books) const {
        if (books.empty()) {
            cout << "No books in the library.\n";
            return;
        }

        cout << "\n==== All Books ====\n";
        for (size_t i = 0; i < books.size(); ++i) {
            cout << "\nBook " << (i + 1) << ":\n";
            books[i]->displayInfo();
            cout << "\n"; // Keep newline after full book info
        }
    }

    void viewAllBorrowRecords(const vector<shared_ptr<BorrowRecord>>& records) const {
        if (records.empty()) {
            cout << "No borrowing records found.\n";
            return;
        }

        cout << "\n==== All Borrowing Records ====\n";
        for (size_t i = 0; i < records.size(); ++i) {
            cout << "\nRecord " << (i + 1) << ":\n";
            records[i]->displayInfo();
            cout << "\n"; // Keep newline after full record info
        }
    }

    // Calls the refactored helper function, marked const
    void searchBooks(const vector<shared_ptr<Book>>& books) const {
        searchBooksHelper(books);
    }

    void viewOverdueBooks(const vector<shared_ptr<BorrowRecord>>& records) const {
        if (records.empty()) {
            cout << "No borrowing records found.\n";
            return;
        }

        bool foundOverdue = false;
        cout << "\n==== Overdue Books ====\n";

        for (const auto& record : records) {
            // Only show records for books that are *currently* marked as borrowed
            // and are overdue. This prevents showing records for returned overdue books.
            if (!record->getBook()->getAvailability() && record->isOverdue()) {
                 cout << "\n"; // Newline before record info
                 record->displayInfo();
                 cout << "\n"; // Newline after record info
                 foundOverdue = true;
            }
        }

        if (!foundOverdue) {
            cout << "No currently borrowed books are overdue.\n";
        }
    }
};

// --- Member class derived from User ---
class Member : public User {
private:
    // Stores records ONLY for books currently borrowed by THIS member
    vector<shared_ptr<BorrowRecord>> borrowedBooks;

public:
    Member(const string& name, int id) : User(name, id) {}

    void displayMenu() const override { // Marked const
        cout << "\n==== Member Menu (" << name << ") ====\n";
        cout << "1. View available books\n";
        cout << "2. Borrow a book\n";
        cout << "3. Return a book\n";
        cout << "4. View my borrowed books\n";
        cout << "5. Search books\n";
        cout << "6. Logout\n";
        // Prompt handled by getValidatedIntInput
    }

    void processChoice(int choice, vector<shared_ptr<Book>>& books,
                       vector<shared_ptr<BorrowRecord>>& allRecords) override {
        switch (choice) {
            case 1: viewAvailableBooks(books); break;
            case 2: borrowBook(books, allRecords); break;
            case 3: returnBook(allRecords); break; // Needs allRecords to find the global record if needed later
            case 4: viewBorrowedBooks(); break;
            case 5: searchBooks(books); break; // Calls refactored helper
            case 6: cout << "Logging out...\n"; break; // Logout handled in main loop
            default: cout << "Invalid choice. Please try again.\n"; break;
        }
    }

    // Getter is const
    const vector<shared_ptr<BorrowRecord>>& getBorrowedBooks() const {
        return borrowedBooks;
    }

private:
    // View methods are const
    void viewAvailableBooks(const vector<shared_ptr<Book>>& books) const {
        if (books.empty()) {
            cout << "No books in the library.\n";
            return;
        }

        bool foundAvailable = false;
        cout << "\n==== Available Books ====\n";

        for (const auto& book : books) { // Use range-based for loop
            if (book->getAvailability()) {
                cout << "\n"; // Newline before book info
                book->displayInfo();
                cout << "\n"; // Newline after book info
                foundAvailable = true;
            }
        }

        if (!foundAvailable) {
            cout << "No books are currently available.\n";
        }
    }

    void borrowBook(vector<shared_ptr<Book>>& books,
                   vector<shared_ptr<BorrowRecord>>& allRecords) {
        if (books.empty()) {
            cout << "No books in the library to borrow.\n";
            return;
        }

        string isbn;
        cout << "Enter ISBN of book to borrow: ";
        getline(cin, isbn);

        auto it = find_if(books.begin(), books.end(),
                          [&isbn](const shared_ptr<Book>& book) {
                              return book->getISBN() == isbn;
                          });

        if (it != books.end()) {
            if ((*it)->getAvailability()) {
                (*it)->setAvailability(false); // Mark book as borrowed
                // Create a new record
                auto record = make_shared<BorrowRecord>(*it, id); // Pass book shared_ptr and member ID
                borrowedBooks.push_back(record); // Add to member's personal list
                allRecords.push_back(record);    // Add to the global list of all records
                cout << "Book borrowed successfully. Due date: "
                      << record->getDueDate().toString() << "\n";
            } else {
                cout << "This book is currently borrowed and not available.\n";
            }
        } else {
            cout << "Book with ISBN " << isbn << " not found.\n";
        }
    }

    void returnBook(vector<shared_ptr<BorrowRecord>>& allRecords) {
        if (borrowedBooks.empty()) {
            cout << "You have not borrowed any books to return.\n";
            return;
        }

        string isbn;
        cout << "Enter ISBN of book to return: ";
        getline(cin, isbn);

        // Find the record in the MEMBER'S list
        auto it = find_if(borrowedBooks.begin(), borrowedBooks.end(),
                          [&isbn](const shared_ptr<BorrowRecord>& record) {
                              return record->getBook()->getISBN() == isbn;
                          });

        if (it != borrowedBooks.end()) {
            // Mark the book as available again
            (*it)->getBook()->setAvailability(true);

            // Remove the record ONLY from the member's list.
            // Keep it in `allRecords` for historical purposes (Librarian view).
            // If strict removal from allRecords is desired, need to search and erase there too.
            borrowedBooks.erase(it);

            cout << "Book returned successfully.\n";
        } else {
            cout << "You have not borrowed a book with ISBN " << isbn << ".\n";
        }
    }

    void viewBorrowedBooks() const { // Marked const
        if (borrowedBooks.empty()) {
            cout << "You have not borrowed any books.\n";
            return;
        }

        cout << "\n==== Your Borrowed Books ====\n";
        for (size_t i = 0; i < borrowedBooks.size(); ++i) {
            cout << "\nBook " << (i + 1) << ":\n";
            borrowedBooks[i]->getBook()->displayInfo(); // Display book details
            cout << "\n  Due Date: " << borrowedBooks[i]->getDueDate().toString();
            if (borrowedBooks[i]->isOverdue()) {
                cout << " (OVERDUE)";
            }
            cout << "\n"; // Newline after full entry
        }
    }

    // Calls the refactored helper function, marked const
    void searchBooks(const vector<shared_ptr<Book>>& books) const {
        searchBooksHelper(books);
    }
};

// --- Main LibrarySystem class ---
class LibrarySystem {
private:
    vector<shared_ptr<Book>> books;
    vector<shared_ptr<BorrowRecord>> borrowRecords; // Global list of all records ever made
    vector<shared_ptr<User>> users;
    User* currentUser = nullptr; // Raw pointer for demonstration (as per requirement Q5.1)

public:
    LibrarySystem() {
        // Add some default users
        users.push_back(make_shared<Librarian>("Admin", 1001)); // Unique IDs recommended
        users.push_back(make_shared<Librarian>("ENG Ahmed Ehab_A", 1002));
        users.push_back(make_shared<Member>("Youssef", 2001));
        users.push_back(make_shared<Member>("ENG Ahmed Ehab_M", 2002));

        // Add some popular Arabic books (titles in English)
        books.push_back(make_shared<Book>("Children of our Alley", "Naguib Mahfouz", "9780385264730"));
        books.push_back(make_shared<Book>("The Trilogy: Palace Walk", "Naguib Mahfouz", "9780385264259"));
        books.push_back(make_shared<Book>("Cities of Salt", "Abdul Rahman Munif", "9780394755267"));
        books.push_back(make_shared<Book>("Season of Migration to the North", "Tayeb Salih", "9780435900311"));
        books.push_back(make_shared<Book>("Memory for Forgetfulness", "Mahmoud Darwish", "9780520087682"));
        books.push_back(make_shared<Book>("The Open Door", "Latifa al-Zayyat", "9789774248122"));
        books.push_back(make_shared<Book>("The Dove's Necklace", "Raja Alem", "9780715649732"));
    }

    void run() {
        int choice;
        bool running = true;

        while (running) {
            if (!currentUser) {
                displayLoginMenu();
                // Use validated input for login choice
                choice = getValidatedIntInput("Enter your choice: ");

                if (choice == 3) {
                    running = false; // Exit
                } else if (choice == 1 || choice == 2) {
                    login(choice); // Attempt login
                } else {
                    cout << "Invalid choice. Please enter 1, 2, or 3.\n";
                }
            } else {
                // User is logged in
                currentUser->displayMenu(); // Display role-specific menu
                // Use validated input for menu choice
                choice = getValidatedIntInput("Enter your choice: ");

                // Check for logout conditions
                bool isLibrarian = dynamic_cast<Librarian*>(currentUser) != nullptr;
                bool isMember = dynamic_cast<Member*>(currentUser) != nullptr;

                if ((isLibrarian && choice == 7) || (isMember && choice == 6)) {
                    logout();
                } else {
                    // Process the choice using polymorphism
                    currentUser->processChoice(choice, books, borrowRecords);
                }
            }
             // Add a small pause or prompt before clearing screen/looping if desired
             // cout << "\nPress Enter to continue...";
             // cin.get(); // Waits for enter key
        }

        cout << "\nThank you for using the Library Management System!\n";
    }

private:
    void displayLoginMenu() const { // Marked const
        cout << "\n==== Library Management System ====\n";
        cout << "1. Login as Librarian\n";
        cout << "2. Login as Member\n";
        cout << "3. Exit\n";
        // Prompt handled by getValidatedIntInput in run()
    }

    void login(int userType) {
        // Use validated input for ID
        int id = getValidatedIntInput("Enter your ID: ");

        bool found = false;
        for (const auto& user : users) {
            if (user->getId() == id) {
                // Check if the found user's type matches the login attempt type
                bool typeMatch = (userType == 1 && dynamic_cast<Librarian*>(user.get())) ||
                                 (userType == 2 && dynamic_cast<Member*>(user.get()));

                if (typeMatch) {
                    currentUser = user.get(); // Assign raw pointer
                    cout << "Login successful. Welcome, " << currentUser->getName() << "!\n";
                    found = true;
                    break; // Exit loop once user is found and logged in
                } else {
                    // ID found, but type mismatch
                    cout << "Login failed: ID found, but role does not match selection.\n";
                    found = true; // Set found to true to prevent "Invalid ID" message
                    break;
                }
            }
        }

        if (!found) {
            cout << "Login failed: Invalid ID.\n";
        }
    }

    void logout() {
        if (currentUser) {
             cout << "Logging out " << currentUser->getName() << ".\n";
             currentUser = nullptr; // Clear the raw pointer
        }
    }
};

// --- Main function ---
int main() {
    LibrarySystem system;
    system.run();
    return 0;
}