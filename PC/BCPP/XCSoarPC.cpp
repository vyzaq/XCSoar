//---------------------------------------------------------------------------
#include <vcl.h>
#pragma hdrstop
USEFORM("Unit1.cpp", Form1);

//#pragma resource "XCSoar.res"
//---------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

int WINAPI xcsoarWinMain(     HINSTANCE hInstance,
                        HINSTANCE hPrevInstance,
                        LPTSTR    lpCmdLine,
                        int       nCmdShow);

#ifdef __cplusplus
}
#endif

#include <stdio.h>

WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
  printf("hello\n");

  xcsoarWinMain(hInstance, hPrevInstance, NULL, nShowCmd);

  return (0);

  try
  {
   Application->Initialize();
   Application->CreateForm(__classid(TForm1), &Form1);
                 Application->Run();
  }
  catch (Exception &exception)
  {
   Application->ShowException(&exception);
  }
  return 0;
}


//---------------------------------------------------------------------------
