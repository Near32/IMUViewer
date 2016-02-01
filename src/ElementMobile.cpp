#include "ElementMobile.h"

ElementMobile::ElementMobile() : IElementMobile()
{

}

ElementMobile::ElementMobile(std::string name_, std::unique_ptr<se3> pose_) : IElementMobile(name_, std::move(pose_) )
{

}

ElementMobile::ElementMobile(std::string name_, std::unique_ptr <se3> pose_, const Mat<float>& hwd_) : IElementMobile(name_,std::move(pose_),hwd_)
{

}

ElementMobile::ElementMobile(std::string name_, se3* pose_, const Mat<float>& hwd_)  : IElementMobile(name_,pose_,hwd_)
{

}

ElementMobile::~ElementMobile()
{

}

bool ElementMobile::isFixe()
{
  return false;
}

bool ElementMobile::isComposit()
{
 	return false;
}



// this function takes no argument at all if you look carefully at the diagram : it is a function which enables the user to know if it tries to access to Fixed Element or a Mobile one. It is made virtual pure in the IElement class because thus each of the daugther classes have to implements it its related way : in the ElementMobile class, this function will return false, since the instance is not an instance of ElementFixe. And in the ElementFixe class, this function will return true.
//thos type of function are very useful when we use the polymorphism quality of the inheritance:
//for instance :
//vector<std::unique_ptr<IElement> > collection;
//that is what we've got in the Environnement class.
//when we want to access the ith element of our container collection, we do not know if it is an instance of ElementMobile or of ElementFixe.
//In order to know, we just have to call that function :
//if( collection[i].isFixe() )
//{
	//we have an instance of ElementFixe
//}
//else
//{
	//we have an instance of ElementMobile.
//}

//INHERITANCE ; POLYMORPHISM ; "abstract" MOTHER CLASS CONTAINER which contains daugther classes.





