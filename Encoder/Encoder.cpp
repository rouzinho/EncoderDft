

// CEDAR INCLUDES
#include "Encoder.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"

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
mNameJoint(new cedar::aux::StringParameter(this, "Joint Name", ""))
{
this->declareOutput("output", mOutput);

mGaussMatrixSizes.push_back(100);
mGaussMatrixSigmas.push_back(3.0);
mGaussMatrixCenters.push_back(25.0);
//init the variable that will get the sensor value
dat = 0;

this->connect(this->mSize.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mSigma.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mValue.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mNameJoint.get(), SIGNAL(valueChanged()), this, SLOT(reName()));


}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
void Encoder::compute(const cedar::proc::Arguments&)
{

  ros::Rate loop_rate(98);
  loop_rate.sleep();
  ros::spinOnce();
  mGaussMatrixCenters.clear();
  mGaussMatrixCenters.push_back(dat);
  //change the Gaussian function with the value of the sensor.
  this->mOutput->setData(cedar::aux::math::gaussMatrix(1,mGaussMatrixSizes,value,mGaussMatrixSigmas,mGaussMatrixCenters,true));

}

void Encoder::reCompute()
{
   mGaussMatrixSizes.clear();
   mGaussMatrixSigmas.clear();
   mGaussMatrixSizes.push_back(static_cast<int>(this->mSize->getValue()));
   mGaussMatrixSigmas.push_back(static_cast<double>(this->mSigma->getValue()));
   value = static_cast<double>(this->mValue->getValue());

}


void Encoder::reName()
{
   topicName = this->mTopic->getValue();
   jointName = this->mNameJoint->getValue();
   const std::string tname = topicName;
   sub = n.subscribe(tname, 1000, &Encoder::chatterCallback,this);
}

//callback for the subscriber. This one get the value of the sensor.
void Encoder::chatterCallback(const sensor_msgs::JointStateConstPtr& state)
{
   //ROS_INFO("I heard: [%f]", state->data);
   //dat = msg->data;
   int i = state->name.size();
   for(int j = 0;j < i;j++)
   {
     if(jointName.compare(state->name[j]) == 0)
     {
       dat = static_cast<double>(state->position[j]);
     }
   }
}

void Encoder::reset()
{

	//ros::shutdown();

}
