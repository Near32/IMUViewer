#include <iostream>
#include <memory>
#include <vector>

class dummy
{
	public :
	
	float val;
	float func()	{	return 1.0f;	}
	
	private :
	
};

using namespace std;

int main(int argc, char* argv[])
{
	vector<unique_ptr<dummy> > collint;
	collint.insert( collint.begin(), unique_ptr<dummy>(new dummy()) );
	collint.insert( collint.begin(), unique_ptr<dummy>(new dummy()) );
	
	vector<unique_ptr<dummy> >::iterator it = collint.begin();
	
	//cout << *it << endl;
	float val = 2 + (*it)->func();
	
	cout << val << endl;
	
	vector<unique_ptr<dummy> >::iterator other = it;
	it++;
	val += (*other)->func();
	
	cout << val << endl;
	
	dummy& test = **it;
	test.val=1.0f;
	cout << test.func() << endl;
	cout << "val = " << (*it)->val << endl;
	
	return 0;
}
