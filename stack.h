/* C++ program to implement basic stack
operations */

using namespace std;

#define MAX 18*18

class Stack {
	char top;

public:
	char a[MAX]; // Maximum size of Stack

	Stack() { top = -1; }
	bool push(char x);
	char pop();
	char peek();
	bool isEmpty();
};

bool Stack::push(char x)
{
	if (top >= (MAX - 1)) {
		// cout << "Stack Overflow";
		return false;
	}
	else {
		a[++top] = x;
		// cout << x << " pushed charo stack\n";
		return true;
	}
}

char Stack::pop()
{
	if (top < 0) {
		// cout << "Stack Underflow";
		return 0;
	}
	else {
		char x = a[top--];
		return x;
	}
}
char Stack::peek()
{
	if (top < 0) {
		// cout << "Stack is Empty";
		return 0;
	}
	else {
		char x = a[top];
		return x;
	}
}

bool Stack::isEmpty()
{
	return (top < 0);
}

// // Driver program to test above functions
// char main()
// {
// 	class Stack s;
// 	s.push(10);
// 	s.push(20);
// 	s.push(30);
// 	cout << s.pop() << " Popped from stack\n";
// 	//prchar all elements in stack :
// 	cout<<"Elements present in stack : ";
// 	while(!s.isEmpty())
// 	{
// 		// prchar top element in stack
// 		cout<<s.peek()<<" ";
// 		// remove top element in stack
// 		s.pop();
// 	}

// 	return 0;
// }
