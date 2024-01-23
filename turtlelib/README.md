# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

      1. Create a helper function as normalize for Vector2D updating the member variables.

      2.  Create a function as part of the struct.

      3. Create a normalize helper function that accepts a reference to a Vector2D and returns a new Vector 2D 

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

      1. Pro: Simple to impliment and is in accordance with guideline C.4

         Con: This would modify the object Vector2D which might not be desired all the time.

      2. Pro: Putting related data and functions together in a struct is better for comprehension, especially since this function take a vector2D input and returns another Vector2D.

         Con: Increases coupling which is not necessarily desired. 

      3. Pro: This has easy access to the Vector2D and reduce coupling. 

         Con: Increases memory requirements as we make a copy.

   - Which of the methods would you implement and why?

     I went with method 3 as it doesn't modify the original Vector2D.

2. What is the difference between a class and a struct in C++?

The difference between a class and a struct in C++ is that class members are private by default, while struct members are public by default.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

Vector2D only has public members where anyone can access its x and y components and modify them so they are defined as structs. On the otherhand, the object variables of Transform2D need to be private members to avoid accidental modification of the vector and rotation angle, hence it are defined as a class. Based on C.8.

Based on C.2, a class should be used if a data structure has an invariant, while a struct should be used if all data members can vary independently. Vector2D has different elements of a vector which are not bound to eachother and can vary independently. The Transform2D data members constitute components of a transformation matrix which is essentially a rotation matrix and a vector creating constraints. Therefore the Transform2D has an invariant, making it more suitable as a class.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

Single argument constructors should be set to explicit by default to avoid any implicit type conversions, Core guideline C.46.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

In the *= operator, it stores the result in the object, meaning the values of the object are changed. To do so they must not be set as const so that they are mutable. While the inv() function is declared const as the transform2D objects values don't changed after calling the function. 
