# Rigid 2D transformation Library
A library for handling transformations in SE(2).

# Conceptual Questions
1. What is the difference between a class and a struct in C++? Member visibility: a class has by default private members, which cannot be accessed by functions outside of the class; the members of a struct can always be accessed (the equivalent of class public members)
2. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specic C++ core guidelines in your answer)?
3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)? Some of the constructors in Transform2D only involve one argument and single-argument constructors are declared explicit to avoid unintended conversions that could cast the object to the specified argument type.
4. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?
5. Why is Transform2D::inv() declared const while Transform2D::operator\*=() is not (Refer to C++ core guidelines in your answer)? By default, member functions should be const unless they are meant to change the object observable state. This is done to ensure that the risk of member functions accidentally modifying an object due to an erroneous implementation or other circumstances is modified. Transform2D::inv() returns a new Transform2D object and therefore does not modify the current object state, and can remain const; Transform2D::operator\*=() directly modifies the object private members and cannot be const.
