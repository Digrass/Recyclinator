#ifndef LEFT_PRESENTER_HPP
#define LEFT_PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <gui/model/ModelCommon.hpp>

using namespace touchgfx;

class LeftView;

class LeftPresenter : public Presenter, public ModelListener
{
public:
    LeftPresenter(LeftView& v);

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

    virtual ~LeftPresenter() {};

    virtual void notifyActionSequenceComplete(OperationDirection actionDir) override;

private:
    LeftPresenter();

    LeftView& view;
};

#endif // LEFT_PRESENTER_HPP
