#ifndef RIGHT_PRESENTER_HPP
#define RIGHT_PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <gui/model/ModelCommon.hpp>

using namespace touchgfx;

class RightView;

class RightPresenter : public Presenter, public ModelListener
{
public:
    RightPresenter(RightView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~RightPresenter() {};

    virtual void notifyActionSequenceComplete(OperationDirection actionDir) override;

private:
    RightPresenter();

    RightView& view;
};

#endif // RIGHT_PRESENTER_HPP
