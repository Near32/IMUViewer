#ifndef MATH_H
#define MATH_H

#include "Mat/Mat.h"
#include "SparseMat/SparseMat.h"
#include "MVG/MVG.h"
#include "Quaternion/Quaternion.h"
#include <vector>
#include <map>
#include <exception>


extern Mat<float> Identity3;

class se3
{
	private :
	
	Mat<float>* w;		//rotational tangent values.
	Mat<float>* t;		//translational values.
	Mat<float>* SE3;		// (R | t) matrix.
	bool hasChanged;
	
	public :
	
	se3();
	se3(const Mat<float>& w_, const Mat<float>& t_);
	se3(const Mat<float>& t_);
	se3(const float* w_t_array);
	se3(const se3& x);
	
	~se3();
	
	Mat<float> exp();					//compute the (R | t) matrix.
	Mat<float> getT()	const;
	Mat<float> getW()	const;
	Mat<float> getSE3()	const;
	void setT(const Mat<float>& t_);
	void setW(const Mat<float>& w_);
	void setOrientation( const Quat& q);	
	
	se3& operator=(const se3& x);
	
	

};



Mat<float> crossproductV( const Mat<float>& p1, const Mat<float>& p2);




/*
typedef struct Coord{
	int ib;
	int jb;
	int ie;
	int je;
	
	Coord( int ib_, int jb_, int ie_, int je_) : ib(ib_), jb(jb_), ie(ie_), je(je_) 
	{
	}
	
} Coord;

template<typename T>
class SparseMat
{
	private :
	int nbrLine;
	int nbrColumn;
	
	std::vector<Mat<T> > blocks;
	std::vector<Coord> coords;
	
	public :
	
	SparseMat(int nbrLine_ = 0, int nbrColumn_ = 0)
	{
		nbrLine = nbrLine_;
		nbrColumn = nbrColumn_;
	}
	
	SparseMat( const Mat<T>& block, const Coord& coord)
	{
		nbrLine = block.getLine();
		nbrColumn = block.getColumn();
		
		blocks.insert( blocks.end(), block);
		coords.insert( coords.end(), coord);
		
	}
	
	SparseMat( const SparseMat<T>& spm)
	{
		nbrLine = spm.getLine();
		nbrColumn = spm.getColumn();
		
		blocks = spm.getBlocks();
		coords = spm.getCoords();	
	}
	
	~SparseMat()
	{
	
	}
	
	
	void add( int i, int j, const T value)
	{
		bool done = false;
		
		for( int b=blocks.size();b--;)
		{
			if( coords[b].ib <= i && coords[b].ie >= i)
			{
				if( coords[b].jb >= j && coords[b].je >= j)
				{
					blocks[b].set( value + blocks[b].get( i-coords[b].ib+1, j-coords[b].jb+1) , i-coords[b].ib+1, j-coords[b].jb+1 );
					b = 0;
					done = true;
					//it is finished.
				}
			}
		}
		
		if(!done)
		{
			//we have to create that corresponding block :
			blocks.insert( blocks.end(), Mat<T>(value,1,1) );
			coords.insert( coords.end(), Coord(i,j,i,j) );
		}
	}
	
	void checkIntegrity()
	{
		std::vector<Mat<T> >::iterator itb = blocks.begin();
		std::vector<Mat<T> >::iterator otherb = itb++;
		std::vector<Coord>::iterator itc = coords.begin();
		std::vector<Coord>::iterator otherc = itc++;
		
		bool change = false;
		
		while( itb != blocks.end() )
		{
			//assume that blocks are settled column-majoredly :
			if( itc.ie+1 == otherc.ib)
			{
				//TODO!!!
			}
			
			if(!change)
			{
				itb++;
				itc++;
			}
		}
	}
	
	//-----------------------------------------------------
	//-----------------------------------------------------
	
	
	inline int getLine()	const
	{
		return nbrLine;
	}
	
	inline int	getColumn()	const
	{
		return nbrColumn();
	}
	
	std::vector<Mat<T> >& getBlocksReference()	const
	{
		return blocks;
	}
	
	std::vector<Coord>& getCoordsReference()	const
	{
		return coords;
	}
	
	std::vector<Mat<T> > getBlocks()	const
	{
		return blocks;
	}
	
	std::vector<Coord> getCoords()	const
	{
		return coords;
	}
	
	inline int getNbrBlocks()	const
	{
		return blocks.size();
	}
	
	Mat<T>& getBlock( int idx)	const
	{
		if( idx > 0 && idx <= blocks.size() )
		{
			return blocks[idx-1];
		}
		else
		{
			cerr << "SparseMat getBlock : wront index." << endl;
		}
		
		return Mat<T>((T)0,1,1);
	}
	
	Coord& getBlockCoord( int idx)	const
	{
		if( idx > 0 && idx <= blocks.size() )
		{
			return coords[idx-1];
		}
		else
		{
			cerr << "SparseMat getBlock : wront index." << endl;
		}
		
		return Coord(0,0,0,0);
	}
};


SparseMat<T> operator=(SparseMat<T>& spm1, SparseMat<T>& spm2)
{
	spm1.SparseMat(spm2);
	return spm1;
}


SparseMat<T> operator*(const SparseMat<T>& spm1, const SparseMat<T>& spm2)
{
	int nbrLine = spm1.getLine();
	int nbrColumn = spm2.getColumn();
	SparseMat<T> ret(nbrLine,nbrColumn);
	int nbrBlocks1 = spm1.getNbrBlocks();
	int nbrBlocks2 = spm2.getNbrBlocks();
		
	if( spm1.getColumn() == spm2.getLine() )
	{
		for(int i=1;i<=nbrLine;i++)
		{
			for(int j=1;j<=nbrColumn;j++)
			{
				T sum = (T)0;
				Coord coordij(i,j,i,j);
				
				for(int b1=1;b1<=nbrBlocks1;b1++)
				{
					Coord tempc1 = spm1.getCoord(b1);
					
					if( tempc1.ib <= i && tempc1.ie >= i)
					{
						Mat<T> bm1( spm1.getBlock(b1) );
						
						for(int b2=1;b2<=nbrBlocks2;b2++)
						{
							Coord tempc2 = spm2.getCoord(b2);
							
							if( tempc2.jb <= j && tempc2.je >= j)
							{
								//let us assert that the transposed block bm1 has line spanning that intersect the one of bm2 :
								if( tempc1.je > tempc2.ib )
								{
									//then those two blocks will contribute to the construction of this cij :
									Mat<T> bm2( spm2.getBlock(b2) );
								
									for( int k=1;k<=bm1.getLine();k++)
									{
										sum+= bm1.get( i-tempc1.ib+1, k)*bm2.get(tempc1.ib+k-1, j-tempc2.jb+1);
									}
									
									ret.add( i, j, sum); 
								}
								
								sum=(T)0;
							}
						
						
						}
					}
				}
			}
		}
		
		ret.checkIntegrity();	
	}
	else
	{
		cerr << "SparseMat Product : wrong dimensions." << endl;
	}	
	
	return ret;
}
*/



#endif
