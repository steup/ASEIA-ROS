#include "Attributes.h"
#include "Nurbs.h"

#include <aseia_car_sim/NurbsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <Transformation.h>

#include <MetaEvent.h>
#include <IO.h>

namespace aseia_car_sim {

  using namespace std;
  using namespace ::id::attribute;

  class UTMToRoadTransformer : public Transformer {
    private:
      dynamic_reconfigure::Server<NurbsConfig> mDynReConfServer;
      uint32_t mInRef, mOutRef;
      MetaValue mRoadLength;
      MetaValue mNurbPos;
      MetaValue mNurbOri;
      MetaEvent mNurbEvent;
      std::vector<MetaNURBCurve::Point> mSamples;
      bool mCurveReady=false;
      size_t mSampleSize = 100;

      void nurbsCoord(MetaValue& posIn, MetaValue& oriIn) {
        posIn -= mNurbPos;
        MetaValue min=MetaValue(1000, id::type::Float::value());
        size_t minI=0;
        ROS_DEBUG_STREAM("Input Position " << posIn);
        for(size_t i=0; i<mSamples.size(); i++) {
          MetaValue temp=(posIn-mSamples[i]).block(0,0,2,1);
          if(temp.norm()<min) {
            minI=i;
            min = temp.norm();
          }
        }
        ROS_DEBUG_STREAM("Fitting Road sample (" << minI << "): " << mSamples[minI] << " difference is " << min << ")");
        MetaValue d = mSamples[minI]-posIn;
        MetaValue n = mSamples[minI]-mSamples[(minI-1)%mSamples.size()];
        MetaValue t = d.dot(n)/n.dot(n);
        if(t>0) {
          n = mSamples[(minI+1)%mSamples.size()]-mSamples[minI];
          t = d.dot(n)/n.dot(n);
        }
        MetaValue offset = (t*n-d).norm();
        MetaValue u({{{0, (t*n).norm().get(0,0)}}}, ((ValueType)offset).typeId());
        offset+=u;
        ROS_DEBUG_STREAM("Lane offset: \n\td = " << d.transpose() << "\n\tn: " << n.transpose() << "\n\to: " << offset);
        posIn = MetaValue({{{0, 0}}, {{0, 0}}, {{(float)minI/mSamples.size(), 1.0/mSamples.size()}}}, posIn.typeId())*mRoadLength;
        if((posIn(0,0)-n(1,0)*posIn(1,0))<0)
          posIn.block(1,0, offset);
        else
          posIn.block(1,0, -offset);
      }

      void dynReConfCallback(NurbsConfig &config, uint32_t level) {
        ROS_INFO_STREAM("Reconfigure UTMToRoad: \n\t sampleSize: " << config.int_param);
        mSampleSize = config.int_param;
        this->operator()(mNurbEvent);
      }

    public:
      UTMToRoadTransformer(const EventType& out, const EventTypes& in)
        : Transformer(out, in) {
        mOutRef = out.attribute(Position::value())->scale().reference();
        mInRef = in[0].attribute(Position::value())->scale().reference();
        dynamic_reconfigure::Server<NurbsConfig>::CallbackType f;
        f = boost::bind(&UTMToRoadTransformer::dynReConfCallback, this, _1, _2);
        mDynReConfServer.setCallback(f);
      }

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
          ROS_DEBUG_STREAM("Got Road description with: " << endl << "\tDimension: " << dim << endl <<"\t#Points: " << pSize << "\t#Knots: " << lSize);
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
          bool first = true;
          MetaValue oldSample;
          for(const auto& p : mSamples) {
            if(!first)
              length += (p-oldSample).norm();
            oldSample = p;
            first=false;
          }
          os << "\tLenght: " << length;
          ROS_DEBUG_STREAM(os.str());
          mRoadLength = move(length);
          mCurveReady = true;
          return {};
        }
        if(attr->scale().reference() == mInRef && mCurveReady) {
          MetaEvent e = event;
          e.attribute(Position::value())->scale().reference(mOutRef);
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
  };

  class UTMToRoad : public Transformation {
    public:
      UTMToRoad() : Transformation(Transformation::Type::attribute, 2, EventID::any) {

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
        //todo handle possible orientation
        EventType reference;
        reference.add(AttributeType(Reference::value(), providedPosAT->value(), providedPosAT->scale(), providedPosAT->unit()));
        reference.add(AttributeType(Orientation::value(), ValueType(type, 4, 1, true), providedPosAT->scale(), Radian()));
        reference.add(*goalTimeAT);
        reference.add(AttributeType(Nurbs::value(), ValueType(type, 100, 4, false), goalPosAT->scale(), providedPosAT->unit()));

        auto input = {orig, reference};
        ROS_DEBUG_STREAM("UTMToRoad [" << goal << ", " << provided << "] -> [" << orig << ", " << reference << "]");
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
