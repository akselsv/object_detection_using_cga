/*
 * =====================================================================================
 *
 *       Filename:  xDiscCollision.cpp
 *
 *    Description:  test for collision within a disc (circle)
 *
 *        Version:  1.0
 *        Created:  02/10/2015 19:14:15
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Pablo Colapinto (), gmail -> wolftype
 *   Organization:
 *
 * =====================================================================================
 */

#include "vsr_app.h"

using namespace vsr;
using namespace vsr::cga;

struct MyApp : App {

  Circle cir = Construct::circle(0,0,0);

  //Some Variables
  bool bReset = false;
  float amt = 0;

  /*-----------------------------------------------------------------------------
   *  Setup Variables
   *-----------------------------------------------------------------------------*/
  void setup(){
    ///Bind Gui
    bindGLV();
    ///Add Variables to GUI
    gui(amt,"amt",-100,100)(bReset,"bReset");

    objectController.attach(&cir);

  }


  /*-----------------------------------------------------------------------------
   *  Draw Routines
   *-----------------------------------------------------------------------------*/
  void onDraw(){

    auto p = calcMouse3D();

    draw(cir,0,1,0);

    auto s = Round::surround(cir);
    auto n = Round::carrier(cir);

    // if mouse point lies within surround of circle
    if ( (p<=s)[0] > 0){
      // print out weight of point
      cout << (p^n).wt() << endl;
      // if absolute weight is greater than error
      if ( fabs( (p^n).wt()) <.00001 ) {
        // draw point
        draw(p,1,0,0);
      }
    }


  }

};


int main(){

  MyApp app;
  app.start();

  return 0;

}
