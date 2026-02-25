#ifndef DOWN_PRESENTER_HPP
#define DOWN_PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <gui/model/ModelCommon.hpp>

using namespace touchgfx;

class DownView;

class DownPresenter : public Presenter, public ModelListener
{
public:
    DownPresenter(DownView& v);

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

    virtual ~DownPresenter() {};

    virtual void notifyActionSequenceComplete(OperationDirection actionDir) override;

private:
    DownPresenter();

    DownView& view;
};

#endif // DOWN_PRESENTER_HPP
