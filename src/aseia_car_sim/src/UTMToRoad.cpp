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
      std::vector<MetaNURBCurve::Point> mSamples;
      bool mCurveReady=false;
      static size_t mSampleSize;

      void nurbsCoord(MetaValue& posIn, MetaValue& oriIn) {
        static MetaValue error((ValueType)posIn);
        posIn -= mNurbPos;
        MetaValue min=MetaValue(1000, id::type::Float::value());
        size_t minI=0;
        posIn=MetaFactory::instance().convert((ValueType)mSamples[0], posIn);
        for(size_t i=0; i<mSamples.size(); i++) {
          MetaValue temp=(posIn-mSamples[i]).block(0,0,2,1);
          if(temp.norm()<min) {
            minI=i;
            min = temp.norm();
          }
        }
        error *=MetaValue({{{0.9999f}}}, posIn.typeId());
        error +=MetaValue({{{0.0001f}}}, posIn.typeId())*(posIn-mSamples[minI]);
        ROS_DEBUG_STREAM_NAMED(transName, "Current error: " << error);
        ROS_DEBUG_STREAM_NAMED(transName, "Fitting Road sample (" << minI << "): " << mSamples[minI] << " difference is " << min << ")");
        size_t pre  = minI?minI-1:mSamples.size()-1;
        size_t post = (minI+1)%mSamples.size();
        const MetaValue& A=mSamples[pre], B=mSamples[minI], C=mSamples[post];
        int offsetFactor;
        MetaValue n,d;
        if((B-A).norm() < (B-C).norm()) {
          d = posIn-A;
          n = B-A;
          offsetFactor = -1;
        } else {
          d = posIn-B;
          n = C-B;
          offsetFactor = 1;
        }
        MetaValue I = n*d.dot(n)/n.dot(n)+B;
        MetaValue offset = (I-B).norm();
        offset+=offsetFactor;
        MetaValue laneOffset = (I-posIn).norm();
        laneOffset += MetaValue({{{0, offset.get(0,0)}}}, laneOffset.typeId());
        MetaValue roadPos({{{1.0f, 1.0f/mSamples.size()}}}, posIn.typeId());
        roadPos*=mRoadLength(minI, 0);
        roadPos+=offset;
        posIn = posIn.zero();
        posIn.set(1,0, laneOffset.get(0,0));
        posIn.set(2,0, roadPos.get(0,0));
        ROS_DEBUG_STREAM_NAMED(transName, "road Pose: " << posIn);
        //MetaValue({{{0, 0}}, {{0, 0}}, {{((float)minI+t.value()(0,0).value())/mSamples.size(), 1.0/mSamples.size()}}}, posIn.typeId())*mRoadLength;
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
          ROS_DEBUG_STREAM_NAMED(transName, "Got Road description with: " << endl << "\tDimension: " << dim << endl <<"\t#Points: " << pSize << "\t#Knots: " << lSize);
          MetaNURBCurve c(dim, move(nurbData.block(1, 3, lSize+1, 1)), move(nurbData.block(1, 0, pSize+1, 3)));
          ostringstream os;
          os << "UTMToRoad: got road nurb description:" << endl;
          mSamples.clear();
          for(size_t i=dim*mSampleSize/lSize;i<mSampleSize-dim*mSampleSize/lSize;i++) {
            mSamples.emplace_back(c.sample(MetaValue((float)i/mSampleSize, id::type::Float::value())).transpose());
            os << mSamples.back() << endl;
          }
          ::id::type::ID dataType = nurbData.typeId();
          MetaValue length(dataType, 1);
          mRoadLength = MetaValue(dataType, mSamples.size());
          bool first = true;
          MetaValue oldSample;
          size_t i=0;
          for(const auto& p : mSamples) {
            if(!first)
              length += (p-oldSample).norm();
            mRoadLength.set(i++, 0, length.get(0,0));
            oldSample = p;
            first=false;
          }
          os << "\tLenght: " << length << "\tSegments: " << mRoadLength;
          ROS_DEBUG_STREAM_NAMED(transName, os.str());
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
        : Transformer(out, in) {
        mOutRef = out.attribute(Position::value())->scale().reference();
        mInRef = in[0].attribute(Position::value())->scale().reference();
        mInstances.push_back(this);
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
        ROS_INFO_STREAM("Reconfigure UTMToRoad: \n\t sampleSize: " << config.int_param);
        mSampleSize = config.int_param;
        UTMToRoadTransformer::updateSampleSize(mSampleSize);
      }
      UTMToRoad() : Transformation(Transformation::Type::attribute, 2, EventID::any) {
        dynamic_reconfigure::Server<NurbsConfig>::CallbackType f;
        f = boost::bind(&UTMToRoad::dynReConfCallback, this, _1, _2);
        mDynReConfServer.setCallback(f);
      }

      virtual EventIDs in(EventID goal) const {
        EventID ref({Position::value(), Time::value(), Orientation::value(), Reference::value(), Nurbs::value()});
        ROS_INFO_STREAM("Querying UTMToRoadTransformation for appropriate Input EventIDs: [" << goal << ", " << ref << "]");
        return EventIDs({goal, ref});
      };

      virtual vector<EventType> in(const EventType& goal, const EventType& provided)  const {
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

      virtual TransPtr create(const EventType& out, const EventTypes& in, const AbstractPolicy& policy) const {
        return TransPtr(new UTMToRoadTransformer(out, in));
      }

      virtual void print(ostream& o) const {
        o << "UTM to Road Transformation";
      }
  };

}

PLUGINLIB_EXPORT_CLASS(aseia_car_sim::UTMToRoad, Transformation)
