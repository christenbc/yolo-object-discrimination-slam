#ifndef SMMAP_H
#define SMMAP_H
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#define SIGHT_INC 1

namespace GMapping {

struct PointAccumulator{
	typedef point<float> FloatPoint;
	/* before 
	PointAccumulator(int i=-1): acc(0,0), n(0), visits(0){assert(i==-1);}
	*/
	/*after begin*/
        PointAccumulator(): acc(0,0), nStruct(0), nNonStruct(0), visits(0){}
        PointAccumulator(int i): acc(0,0), nStruct(0), nNonStruct(0), visits(0){assert(i==-1);}
	/*after end*/
        inline void update(bool value, bool nonStruct, const Point& p=Point(0,0));
        inline Point mean() const {return 1./(nStruct+nNonStruct)*Point(acc.x, acc.y);}
        inline operator double() const {
            int n = nStruct + nNonStruct;
            return  visits ? double(n) * SIGHT_INC / double(visits) : -1.0;
        }
        inline double nonStructOccupancy() const {
            int n = nStruct + nNonStruct;
            if (n > 0) {
                return double(nNonStruct) / double(n);
            }
            else {
                return 0.0;
            }
        }
        inline void add(const PointAccumulator& p) {acc=acc+p.acc; nStruct+=p.nStruct; nNonStruct+=p.nNonStruct; visits+=p.visits; }
	static const PointAccumulator& Unknown();
	static PointAccumulator* unknown_ptr;
	FloatPoint acc;
        int nStruct, nNonStruct, visits;
	inline double entropy() const;
};

void PointAccumulator::update(bool value, bool nonStruct, const Point& p){
	if (value) {
		acc.x+= static_cast<float>(p.x);
		acc.y+= static_cast<float>(p.y); 
		visits+=SIGHT_INC;
                if (nonStruct)
                    nNonStruct++;
                else {
                    nStruct++;
                }
        } else
		visits++;
}

double PointAccumulator::entropy() const{
	if (!visits)
		return -log(.5);
        int n = nStruct + nNonStruct;
        if (n==visits || n==0)
		return 0;
        double x=(double)n*SIGHT_INC/(double)visits;
	return -( x*log(x)+ (1-x)*log(1-x) );
}


typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;

};

#endif 
