

// CEDAR INCLUDES
#include "Encoder.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"
#include <cmath>
#include <iostream>

// SYSTEM INCLUDES

//----------------------------------------------------------------------------------------------------------------------
// constructors and destructor
//----------------------------------------------------------------------------------------------------------------------
Encoder::Encoder()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat::zeros(1, 100, CV_32F))),
mSize(new cedar::aux::IntParameter(this, "Size",100)),
mValue(new cedar::aux::DoubleParameter(this, "Amplitude",1.0)),
mSigma(new cedar::aux::DoubleParameter(this,"Sigma",3.0)),
mTopic(new cedar::aux::StringParameter(this, "Topic Name", "")),
mNameJoint(new cedar::aux::StringParameter(this, "Joint Name", "")),
mLower(new cedar::aux::DoubleParameter(this,"lower",-1.0)),
mUpper(new cedar::aux::DoubleParameter(this,"upper",1.0))
{
   this->declareOutput("output", mOutput);

   mGaussMatrixSizes.push_back(100);
   mGaussMatrixSigmas.push_back(3.0);
   mGaussMatrixCenters.push_back(25.0);
   //init the variable that will get the sensor value
   dat = 0;
   old_dat = 0.5;
   value = 1.0;
   size = 100;
   lower_bound = -1.0;
   upper_bound = 1.0;
   output = cedar::aux::math::gaussMatrix(1,mGaussMatrixSizes,value,mGaussMatrixSigmas,mGaussMatrixCenters,true);

   //ros::Rate loop_rate(40);
   //loop_rate.sleep();
   //ros::spinOnce();



   this->connect(this->mSize.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
   this->connect(this->mSigma.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
   this->connect(this->mValue.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
   this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
   this->connect(this->mNameJoint.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
   this->connect(this->mLower.get(), SIGNAL(valueChanged()), this, SLOT(reBound()));
   this->connect(this->mUpper.get(), SIGNAL(valueChanged()), this, SLOT(reBound()));


}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------

// Generate a gaussian curve based on the inputs of the encoder
void Encoder::compute(const cedar::proc::Arguments&)
{
   ros::Rate loop_rate(10);



/*
  if(std::abs((std::abs(old_dat) - std::abs(dat))) > 0.02)
  {
     //std::cout << "---CHANGE---" << '\n';
     old_dat = dat;
     new_dat = this->setPosition(dat);
     mGaussMatrixCenters.clear();
     mGaussMatrixCenters.push_back(new_dat);
     //change the Gaussian function with the value of the sensor.
     output = cedar::aux::math::gaussMatrix(1,mGaussMatrixSizes,value,mGaussMatrixSigmas,mGaussMatrixCenters,true);
  }*/
   new_dat = this->setPosition(dat);
   //std::cout << new_dat << '\n';
   mGaussMatrixCenters.clear();
   mGaussMatrixCenters.push_back(new_dat);
   //change the Gaussian function with the value of the sensor.
   output = cedar::aux::math::gaussMatrix(1,mGaussMatrixSizes,value,mGaussMatrixSigmas,mGaussMatrixCenters,true);
   this->mOutput->setData(output);

   ros::spinOnce();
   //loop_rate.sleep();
   //ros::Duration d = ros::Duration(2, 0);
   //d.sleep();

}

double Encoder::setPosition(double data)
{
   if(lower_bound < 0)
   {
      shift = std::abs(lower_bound);
      new_upper = std::abs(upper_bound) + shift;
      new_data = data + shift;
      tmp = (new_data * size) / new_upper;
   }
   if(lower_bound >= 0)
   {
      tmp = (data * size) / upper_bound;
   }

   return tmp;
}

void Encoder::reCompute()
{
   mGaussMatrixSizes.clear();
   mGaussMatrixSigmas.clear();
   size = static_cast<int>(this->mSize->getValue());
   mGaussMatrixSizes.push_back(size);
   mGaussMatrixSigmas.push_back(static_cast<double>(this->mSigma->getValue()));
   value = static_cast<double>(this->mValue->getValue());
}

void Encoder::reBound()
{
   lower_bound = static_cast<double>(this->mLower->getValue());
   upper_bound = static_cast<double>(this->mUpper->getValue());
}

void Encoder::reName()
{
   topicName = this->mTopic->getValue();
   jointName = this->mNameJoint->getValue();
   const std::string tname = topicName;
   sub = n.subscribe(tname, 1000, &Encoder::chatterCallback,this);
   //ros::Timer timer = n.createTimer(ros::Duration(1), &Encoder::timerCallback,this);

}

//callback for the subscriber. This one get the value of the sensor.
void Encoder::chatterCallback(const sensor_msgs::JointStateConstPtr& state)
{

   //dat = msg->data;
   int i = state->name.size();
   for(int j = 0;j < i;j++)
   {
     if(jointName.compare(state->name[j]) == 0)
     {
       dat = static_cast<double>(state->position[j]);
       //ROS_INFO("I heard: [%f]", dat);
     }
   }
}

void Encoder::reset()
{

	//ros::shutdown();

}
