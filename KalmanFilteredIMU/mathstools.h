/*
 * matrixtools.h
 *
 * Created: 09/01/2012 12:49:34
 *  Authors: Loïc Kaemmerlen, Pascal Madesclair
 * 
 */ 


#ifndef MATRIXTOOLS_H_
#define MATRIXTOOLS_H_

#include <stdlib.h> 
#include <stdio.h>



typedef struct vector
{
//	float x, y, z;
	 float x, y, z;
} vector;

extern void vector_cross(const vector *a, const vector *b, vector *out);
extern float vector_dot(const vector *a, const vector *b);
extern void vector_normalize(vector *a);




class Matrix
{

	public:
	int row;
	int col;
	float **mat;
	
	// Constructor
	void init ();
	Matrix();
	Matrix(const Matrix& m);
	Matrix(int rownum , int colnum);
	
	// Destructor
	~Matrix();
	
	//Operator
	// Allocation
	Matrix& operator =(const Matrix& Other);
	// Addition
	friend Matrix operator+(Matrix const& a, Matrix const& b);
	// Substraction
	friend Matrix operator-(Matrix const& a, Matrix const& b);
	// Multiplication
	friend Matrix operator*(Matrix const& a, Matrix const& b);
	// Call for Matrix values (ex: A(0,1))
	float& operator() (unsigned row, unsigned col);
	
	
	// Functions
	// Returns the inverse of a 3x3 matrix
	Matrix invert_3x3 ();
	
	// Prints on serial the matrix
	void usart_Send_matrix (void);
	
	
	
	
};
















#endif /* MATRIXTOOLS_H_ */