/*
 * ThridPoly.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: seungsu
 */

#include "ThirdPoly.h"


ThirdPoly::ThirdPoly(int dim)
{
    mDim = dim;

    mInitPos.Resize(mDim);
    mInitVel.Resize(mDim);
    mEndPos.Resize(mDim);
    mEndVel.Resize(mDim);

    mParamA.Resize(mDim);
    mParamB.Resize(mDim);
    mParamC.Resize(mDim);
    mParamD.Resize(mDim);
}


void ThirdPoly::SetConstraints(MathLib::Vector &initPos, MathLib::Vector &initVel, MathLib::Vector &endPos, MathLib::Vector &endVel, double duration)
{
    mInitPos.Set(initPos.Array(), mDim);
    mInitVel.Set(initVel.Array(), mDim);
    mEndPos.Set(endPos.Array(), mDim);
    mEndVel.Set(endVel.Array(), mDim);

    // calculate polynomial
//    mParamA = mInitPos*( 2.0) +mEndPos*(-2.0) +mInitVel        +mEndVel;
//    mParamB = mInitPos*(-3.0) +mEndPos*( 3.0) +mInitVel*(-2.0) -mEndVel;
//    mParamC = mInitVel;
//    mParamD = mInitPos;


//    mParamB = (endPos*(3.0) - endVel + initVel(1-duration*(3.0)) - initPos*(3.0))*(1/(duration*(duration*(3.0) - 2.0)));
//    mParamA = (endVel - initVel - mParamB*duration*(2.0))/(duration*duration*duration*3.0);
//    mParamC = mInitVel;
//    mParamD = mInitPos;

    mParamA = ((initPos-endPos)*(2.0) + (initVel + endVel)*duration)/(duration*duration*duration);
    mParamB = ((initPos-endPos)*(-3.0) - (initVel*(2.0) + endVel)*duration)/(duration*duration);
    mParamC = mInitVel;
    mParamD = mInitPos;


    mDuration = duration;


    //cout<<"desired vel = "<<endVel<<endl;
}

void ThirdPoly::Get(double t, MathLib::Vector &pos)
{


    //double lx = t/mDuration;
    double lx = t;
        //cout<<lx<<endl;
        pos = mParamA*lx*lx*lx + mParamB*lx*lx + mParamC*lx + mParamD;


}

void ThirdPoly::Get(double t, MathLib::Vector &pos, MathLib::Vector &vel)
{
    //double lx = t/mDuration;
    double lx = t;

    //cout<<lx<<endl;


    pos = mParamA*lx*lx*lx  + mParamB*lx*lx  + mParamC*lx + mParamD;
    vel = mParamA*lx*lx*3.0 + mParamB*lx*2.0 + mParamC;

    //cout<<"pos "<<pos<<endl;


}

