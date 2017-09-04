#include "Attributes.h"
#include "Nurbs.h"

#include <aseia_car_sim/NurbsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>
#include <MetaFactory.h>
#include <IO.h>


namespace aseia_car_sim {

static const char* transName = "utm_to_road";

  using namespace std;
  using namespace ::id::attribute;

  class UTMToRoadTransformer : public Transformer {
    private:
      static vector<UTMToRoadTransformer*> mInstances;
      uint32_t mInRef, mOutRef;
      MetaValue mRoadLength;
      MetaValue mNurbPos;
      MetaValue mNurbOri;
      MetaEvent mNurbEvent;
      MetaValue mSamples;
      bool mCurveReady=false;
      static size_t mSampleSize;
      ::id::type::ID mPosType, mSampleType;

      void nurbsCoord(MetaValue& posIn, MetaValue& oriIn) {
        ostringstream os;
        posIn -= mNurbPos;
        ValueType helperType = (ValueType)mSamples;
        helperType.rows(1);
        MetaValue diff = (posIn.cast(mSampleType)*MetaValue(helperType).ones()-mSamples);
        diff=diff.cwiseDot(diff);
        int minI=(size_t)diff.transpose().argmin().get(0,0).value();
        MetaValue roadPosError = diff.block(0, minI, 1, 1).zeroValue();
        os << "Fitting Road sample (" << minI << "): " << mSamples.block(0, minI, 3, 1) << " difference is " << diff(0, minI) << ")" << endl;
        int pre  = minI?minI-1:mSamples.cols()-1;
        int post = (minI+1)%mSamples.cols();
        MetaValue A=mSamples.block(0, pre, 3, 1), B=mSamples.block(0, minI, 3, 1), C=mSamples.block(0, post, 3, 1);
        MetaValue offsetFactor(mSampleType, 1, 1, false);
        MetaValue n,d;
        if((B-A).norm() < (B-C).norm()) {
          d = posIn-A;
          n = B-A;
          offsetFactor = -offsetFactor.ones();
        } else {
          d = posIn-B;
          n = C-B;
          offsetFactor = offsetFactor.ones();
        }
        MetaValue I = n*d.dot(n)/n.dot(n)+B;
        os << "n: " << n << endl;
        os << "A: " << A << endl;
        os << "B: " << B << endl;
        os << "C: " << C << endl;
        os << "I: " << I << endl;
        MetaValue offset = (I-B).block(0,0,2,0).norm()*offsetFactor;
        MetaValue laneOffset = (I-posIn).norm();
        laneOffset += MetaValue({{{0, offset.get(0,0)}}}, laneOffset.typeId());
        MetaValue roadPos({{{1.0f, 1.0f/(mSamples.cols())}}}, posIn.typeId());
        roadPos*=mRoadLength(0, minI);
        roadPos+=offset;
        posIn.set(0,0,0);
        posIn.block(1,0, move(laneOffset));
        posIn.block(2,0, move(roadPos));
        os << "road offset: " << roadPos << " road offset error: " << roadPosError << endl;
        os << "road Pose: " << posIn << " forwarded error: " << roadPosError << endl;
        os << "road Pose Type: " << (ValueType)posIn << " road offset Type: " << (ValueType)roadPos << endl;
        ROS_DEBUG_STREAM_NAMED(transName, os.str());
        posIn+=posIn.ones()*roadPosError;

      }



    public:

      virtual bool check(const MetaEvent& event) const {
        const MetaAttribute* attr = event.attribute(Position::value());
        if(!attr)
          return false;
        if(attr->scale().reference() == mInRef)
          return true;
        if(attr->scale().reference() == mOutRef)
          return true;
        return false;
      }

      virtual Events operator()(const MetaEvent& event) {
        const MetaAttribute* attr = event.attribute(Position::value());
        if(!attr)
          return {};
        if(attr->scale().reference() == mOutRef) {
          mCurveReady = true;
          const MetaAttribute* mPosPtr = event.attribute(Reference::value());
          const MetaAttribute* mOriPtr = event.attribute(Orientation::value());
          const MetaAttribute* mNurbPtr = event.attribute(Nurbs::value());
          if(!mPosPtr || !mOriPtr || !mNurbPtr) {
            ROS_ERROR_STREAM("Invalid Road reference received: dropping it!");
            return {};
          }
          mNurbEvent = event;
          mNurbPos = mPosPtr->value();
          mNurbOri = mOriPtr->value();
          const MetaValue& nurbData = mNurbPtr->value();
          const size_t dim = (size_t)nurbData.get(0,0);
          const size_t pSize = (size_t)nurbData.get(0,1);
          const size_t lSize = (size_t)nurbData.get(0,2);
          ROS_INFO_STREAM_NAMED(transName, "Got Road description with: " << endl << "\tDimension: " << dim << endl <<"\t#Points: " << pSize << "\t#Knots: " << lSize);
          MetaNURBCurve c(dim, move(nurbData.block(1, 3, lSize+1, 1)), move(nurbData.block(1, 0, pSize+1, 3)));
          ostringstream os, debug;
          os << "UTMToRoad: got road nurb description:" << endl;
          mSamples=MetaValue(mSampleType, dim, mSampleSize-2*dim*mSampleSize/lSize, false);
          debug << "mSampleType: " << mSampleType << endl << "mSamples: " << mSamples << endl;
          for(size_t i=0;i<mSampleSize-2*dim*mSampleSize/lSize;i++)
            mSamples.block(0,i,c.sample(MetaValue((float)(i+dim*mSampleSize/lSize)/mSampleSize, mSampleType)).transpose());
          debug << "mSamples: " << mSamples << endl;
          MetaValue shifted = mSamples.block(0, 1, 3, mSamples.cols()-2);
          debug << "shifted: " << shifted << endl;
          mRoadLength = shifted-mSamples.block(0, 0, 3, mSamples.cols()-2);
          debug << "mRoadLenght: " << mRoadLength << endl;
          mRoadLength=mRoadLength.cwiseDot(mRoadLength).sqrt();
          for(size_t i=1;i<mRoadLength.cols();i++)
            mRoadLength.block(0, i, mRoadLength(0, i)+=mRoadLength(0, i-1));
          os << "\tLenght: " << mRoadLength(0, mRoadLength.cols()-1) << "\tSegments: " << mRoadLength;
          ROS_INFO_STREAM_NAMED(transName, os.str());
          ROS_DEBUG_STREAM_NAMED(transName, debug.str());
          if(mSamples.cols() && mRoadLength.cols())
            mCurveReady = true;
          return {};
        }
        if(attr->scale().reference() == mInRef && mCurveReady) {
          MetaEvent e = event;
          e.attribute(Position::value())->scale().reference(mOutRef);
          e.attribute(Time::value())->scale().reference(mOutRef);
          MetaValue& in     = e.attribute(Position::value())->value();
          MetaValue& oriIn  = e.attribute(Orientation::value())->value();
          nurbsCoord(in, oriIn);
          return {e};
        }
        return {};
      }

      virtual  void print(ostream& o) const {
        o << "UTM("  << mInRef << ") to Road(" << mOutRef << ") Transformer";
      }

      static void updateSampleSize(size_t newSampleSize) {
        mSampleSize = newSampleSize;
        for(UTMToRoadTransformer* ptr : mInstances)
          ptr->operator()(ptr->mNurbEvent);
      }

      UTMToRoadTransformer(const EventType& out, const EventTypes& in)
        : Transformer(out, in), mPosType(out[Position()].value().typeId()) {
        mOutRef = out.attribute(Position::value())->scale().reference();
        mInRef = in[0].attribute(Position::value())->scale().reference();
        mInstances.push_back(this);
        for(const EventType& eT: in)
          if(eT.attribute(Nurbs())!=nullptr)
            mSampleType = eT[Nurbs()].value().typeId();
      }

      virtual ~UTMToRoadTransformer() {
        mInstances.erase(remove(mInstances.begin(), mInstances.end(), this), mInstances.end());
      }

  };

  size_t UTMToRoadTransformer::mSampleSize;
  vector<UTMToRoadTransformer*> UTMToRoadTransformer::mInstances;

  class UTMToRoad : public Transformation {
    private:
      dynamic_reconfigure::Server<NurbsConfig> mDynReConfServer;
      size_t mSampleSize;
    public:
      void dynReConfCallback(NurbsConfig &config, uint32_t level) {
        ROS_INFO_STREAM("Reconfigure UTMToRoad: \n\t sampleSize: " << config.nurbSampleSize);
        mSampleSize = config.nurbSampleSize;
        UTMToRoadTransformer::updateSampleSize(mSampleSize);
      }
      UTMToRoad() : Transformation(Transformation::Type::attribute, 2, EventID::any) {
        dynamic_reconfigure::Server<NurbsConfig>::CallbackType f;
        f = boost::bind(&UTMToRoad::dynReConfCallback, this, _1, _2);
        mDynReConfServer.setCallback(f);
      }

      virtual EventIDs in(EventID goal, const MetaFilter& = MetaFilter()) const {
        EventID ref({Position::value(), Time::value(), Orientation::value(), Reference::value(), Nurbs::value()});
        ROS_INFO_STREAM("Querying UTMToRoadTransformation for appropriate Input EventIDs: [" << goal << ", " << ref << "]");
        return EventIDs({goal, ref});
      };

      virtual vector<EventType> in(const EventType& goal, const EventType& provided, const MetaFilter& = MetaFilter())  const {
        if(provided==EventType())
          return {};
        const AttributeType* goalTimeAT = goal.attribute(Time::value());
        const AttributeType* goalPosAT = goal.attribute(Position::value());
        const AttributeType* providedPosAT = provided.attribute(Position::value());
        if(!goalPosAT || !goalTimeAT || !providedPosAT
           || goalPosAT->scale().reference() == providedPosAT->scale().reference())
          return {};
        ::id::type::ID type = ValueType(goalPosAT->value()).typeId();

        EventType orig = goal;
        orig.attribute(Position::value())->scale().reference(providedPosAT->scale().reference());
        orig.attribute(Time::value())->scale().reference(providedPosAT->scale().reference());
        //todo handle possible orientation
        EventType reference;
        reference.add(AttributeType(Reference::value(), providedPosAT->value(), providedPosAT->scale(), providedPosAT->unit()));
        reference.add(AttributeType(Orientation::value(), ValueType(type, 4, 1, true), providedPosAT->scale(), Radian()));
        reference.add(provided[Time()]);
        reference.add(AttributeType(Nurbs::value(), ValueType(type, 100, 4, false), goalPosAT->scale(), providedPosAT->unit()));

        auto input = {orig, reference};
        ROS_DEBUG_STREAM_NAMED(transName, "UTMToRoad [" << goal << ", " << provided << "] -> [" << orig << ", " << reference << "]");
        return input;
      }

      virtual TransPtr create(const EventType& out, const EventTypes& in, const AbstractPolicy& policy, const MetaFilter& = MetaFilter()) const {
        return TransPtr(new UTMToRoadTransformer(out, in));
      }

      virtual void print(ostream& o) const {
        o << "UTM to Road Transformation";
      }
  };

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::UTMToRoad, Transformation)
