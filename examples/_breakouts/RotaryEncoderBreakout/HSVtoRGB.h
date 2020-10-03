#ifndef _HSVTORGB_h
#define _HSVTORGB_h

////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
void HSVtoRGB(float hue, float sat, float val, byte& rOut, byte& gOut, byte& bOut)
{
  ////////////////////////////////
  // Convert HSV values to RGB
  // H(Hue): 0-360 degrees
  // S(Saturation): 0-100 percent
  // V(Value): 0-100 percent
  //
  // RGB out is in range 0->255.
  // This method was found at:
  // https://www.codespeedy.com/hsv-to-rgb-in-cpp/
  ////////////////////////////////
  if(hue > 360.0f || hue < 0.0f || sat > 100.0f || sat < 0.0f || val > 100.0f || val < 0.0f)
  {
    //The given HSV values are not in valid range
    rOut = 0;
    gOut = 0;
    bOut = 0;
    return;
  }
  
  float s = sat / 100.0f;
  float v = val / 100.0f;
  float c = s * v;
  float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
  float m = v - c;
  
  float r, g, b;
  if(hue >= 0.0f && hue < 60.0f)
      r = c, g = x, b = 0.0f;
  else if(hue >= 60.0f && hue < 120.0f)
      r = x, g = c, b = 0.0f;
  else if(hue >= 120.0f && hue < 180.0f)
      r = 0.0f, g = c, b = x;
  else if(hue >= 180.0f && hue < 240.0f)
      r = 0.0f, g = x, b = c;
  else if(hue >= 240.0f && hue < 300.0f)
      r = x, g = 0.0f, b = c;
  else
      r = c, g = 0.0f, b = x;

  rOut = (byte)((r + m) * 255.0f);
  gOut = (byte)((g + m) * 255.0f);
  bOut = (byte)((b + m) * 255.0f);
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
