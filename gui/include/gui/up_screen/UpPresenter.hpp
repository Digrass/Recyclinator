#ifndef UP_PRESENTER_HPP
#define UP_PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <gui/model/ModelCommon.hpp>

using namespace touchgfx;

class UpView;

class UpPresenter : public Presenter, public ModelListener
{
public:
    UpPresenter(UpView& v);

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

    virtual ~UpPresenter() {};

    virtual void notifyActionSequenceComplete(OperationDirection actionDir) override;

private:
    UpPresenter();

    UpView& view;
};

#endif // UP_PRESENTER_HPP
