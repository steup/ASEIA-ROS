#include <TransformationFactory.h>

#include <vector>

class TransformationFactoryImpl : public TransformationFactory{
  private
    std::vector<Transformation*()(void)> mTrans;
  public:
    virtual TransID registerCreator(TransPtr(tCreate)(void)) {
      mTrans.push_back(tCreate);
      return mTrans.size()-1;
    }
    virtual TransPtr create(TransID trans) const {
      return TransPtr(mTrans[trans]());
    }
};
