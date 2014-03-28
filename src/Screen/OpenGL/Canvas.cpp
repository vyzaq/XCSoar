/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2014 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Canvas.hpp"
#include "Triangulate.hpp"
#include "Globals.hpp"
#include "Texture.hpp"
#include "Scope.hpp"
#include "VertexArray.hpp"
#include "Shapes.hpp"
#include "Buffer.hpp"
#include "Features.hpp"
#include "VertexPointer.hpp"
#include "Screen/Custom/Cache.hpp"
#include "Screen/Bitmap.hpp"
#include "Screen/Util.hpp"
#include "Util/AllocatedArray.hpp"
#include "Util/Macros.hpp"

#ifdef HAVE_GLES2
#include "Shaders.hpp"
#include "Program.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#else
#include "Compatibility.hpp"
#endif

#ifndef NDEBUG
#include "Util/UTF8.hpp"
#endif

#include <assert.h>

AllocatedArray<RasterPoint> Canvas::vertex_buffer;

void
Canvas::DrawFilledRectangle(int left, int top, int right, int bottom,
                            const Color color)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  color.Set();

#ifdef HAVE_GLES
  const RasterPoint vertices[] = {
    { left, top },
    { right, top },
    { left, bottom },
    { right, bottom },
  };

  const ScopeVertexPointer vp(vertices);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
#else
  glRecti(left, top, right, bottom);
#endif
}

void
Canvas::OutlineRectangleGL(int left, int top, int right, int bottom)
{
  const ExactRasterPoint vertices[] = {
    RasterPoint{left, top},
    RasterPoint{right, top},
    RasterPoint{right, bottom},
    RasterPoint{left, bottom},
  };

  const ScopeVertexPointer vp(vertices);
  glDrawArrays(GL_LINE_LOOP, 0, 4);
}

void
Canvas::FadeToWhite(GLubyte alpha)
{
  const GLBlend blend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  const Color color(0xff, 0xff, 0xff, alpha);
  Clear(color);
}

void
Canvas::FadeToWhite(PixelRect rc, GLubyte alpha)
{
  const GLBlend blend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  const Color color(0xff, 0xff, 0xff, alpha);
  DrawFilledRectangle(rc.left, rc.right, rc.right, rc.bottom, color);
}

void
Canvas::DrawRaisedEdge(PixelRect &rc)
{
  Pen bright(1, Color(240, 240, 240));
  Select(bright);
  DrawTwoLinesExact(rc.left, rc.bottom - 2, rc.left, rc.top,
                    rc.right - 2, rc.top);

  Pen dark(1, Color(128, 128, 128));
  Select(dark);
  DrawTwoLinesExact(rc.left + 1, rc.bottom - 1, rc.right - 1, rc.bottom - 1,
                    rc.right - 1, rc.top + 1);

  ++rc.left;
  ++rc.top;
  --rc.right;
  --rc.bottom;
}

void
Canvas::DrawPolyline(const RasterPoint *points, unsigned num_points)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  pen.Bind();

  const ScopeVertexPointer vp(points);
  glDrawArrays(GL_LINE_STRIP, 0, num_points);

  pen.Unbind();
}

void
Canvas::DrawPolygon(const RasterPoint *points, unsigned num_points)
{
  if (brush.IsHollow() && !pen.IsDefined())
    return;

#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  ScopeVertexPointer vp(points);

  if (!brush.IsHollow() && num_points >= 3) {
    brush.Set();

    static AllocatedArray<GLushort> triangle_buffer;
    unsigned idx_count = PolygonToTriangles(points, num_points,
                                            triangle_buffer);
    if (idx_count > 0)
      glDrawElements(GL_TRIANGLES, idx_count, GL_UNSIGNED_SHORT,
                     triangle_buffer.begin());
  }

  if (IsPenOverBrush()) {
    pen.Bind();

    if (pen.GetWidth() <= 2) {
      glDrawArrays(GL_LINE_LOOP, 0, num_points);
    } else {
      unsigned vertices = LineToTriangles(points, num_points, vertex_buffer,
                                          pen.GetWidth(), true);
      if (vertices > 0) {
        vp.Update(vertex_buffer.begin());
        glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices);
      }
    }

    pen.Unbind();
  }
}

void
Canvas::DrawTriangleFan(const RasterPoint *points, unsigned num_points)
{
  if (brush.IsHollow() && !pen.IsDefined())
    return;

#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  ScopeVertexPointer vp(points);

  if (!brush.IsHollow() && num_points >= 3) {
    brush.Set();
    glDrawArrays(GL_TRIANGLE_FAN, 0, num_points);
  }

  if (IsPenOverBrush()) {
    pen.Bind();

    if (pen.GetWidth() <= 2) {
      glDrawArrays(GL_LINE_LOOP, 0, num_points);
    } else {
      unsigned vertices = LineToTriangles(points, num_points, vertex_buffer,
                                          pen.GetWidth(), true);
      if (vertices > 0) {
        vp.Update(vertex_buffer.begin());
        glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices);
      }
    }

    pen.Unbind();
  }
}

void
Canvas::DrawHLine(int x1, int x2, int y, Color color)
{
  color.Set();

  const RasterPoint v[] = {
    { GLvalue(x1), GLvalue(y) },
    { GLvalue(x2), GLvalue(y) },
  };

  const ScopeVertexPointer vp(v);
  glDrawArrays(GL_LINE_STRIP, 0, ARRAY_SIZE(v));
}

void
Canvas::DrawLine(int ax, int ay, int bx, int by)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  pen.Bind();

  const RasterPoint v[] = {
    { GLvalue(ax), GLvalue(ay) },
    { GLvalue(bx), GLvalue(by) },
  };

  const ScopeVertexPointer vp(v);
  glDrawArrays(GL_LINE_STRIP, 0, ARRAY_SIZE(v));

  pen.Unbind();
}

void
Canvas::DrawExactLine(int ax, int ay, int bx, int by)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  pen.Bind();

  const ExactRasterPoint v[] = {
    { ToGLexact(ax), ToGLexact(ay) },
    { ToGLexact(bx), ToGLexact(by) },
  };

  const ScopeVertexPointer vp(v);
  glDrawArrays(GL_LINE_STRIP, 0, ARRAY_SIZE(v));

  pen.Unbind();
}

/**
 * Draw a line from a to b, using triangle caps if pen-size > 2 to hide
 * gaps between consecutive lines.
 */
void
Canvas::DrawLinePiece(const RasterPoint a, const RasterPoint b)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  pen.Bind();

  const RasterPoint v[] = { {a.x, a.y}, {b.x, b.y} };
  if (pen.GetWidth() > 2) {
    unsigned strip_len = LineToTriangles(v, 2, vertex_buffer, pen.GetWidth(),
                                         false, true);
    if (strip_len > 0) {
      const ScopeVertexPointer vp(vertex_buffer.begin());
      glDrawArrays(GL_TRIANGLE_STRIP, 0, strip_len);
    }
  } else {
    const ScopeVertexPointer vp(v);
    glDrawArrays(GL_LINE_STRIP, 0, 2);
  }

  pen.Unbind();
}

void
Canvas::DrawTwoLines(int ax, int ay, int bx, int by, int cx, int cy)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  pen.Bind();

  const RasterPoint v[] = {
    { GLvalue(ax), GLvalue(ay) },
    { GLvalue(bx), GLvalue(by) },
    { GLvalue(cx), GLvalue(cy) },
  };

  const ScopeVertexPointer vp(v);
  glDrawArrays(GL_LINE_STRIP, 0, ARRAY_SIZE(v));

  pen.Unbind();
}

void
Canvas::DrawTwoLinesExact(int ax, int ay, int bx, int by, int cx, int cy)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  pen.Bind();

  const ExactRasterPoint v[] = {
    { ToGLexact(ax), ToGLexact(ay) },
    { ToGLexact(bx), ToGLexact(by) },
    { ToGLexact(cx), ToGLexact(cy) },
  };

  const ScopeVertexPointer vp(v);
  glDrawArrays(GL_LINE_STRIP, 0, ARRAY_SIZE(v));

  pen.Unbind();
}

void
Canvas::DrawCircle(int x, int y, unsigned radius)
{
#ifdef HAVE_GLES2
  OpenGL::solid_shader->Use();
#endif

  if (IsPenOverBrush() && pen.GetWidth() > 2) {
    ScopeVertexPointer vp;
    GLDonutVertices vertices(x, y,
                             radius - pen.GetWidth() / 2,
                             radius + pen.GetWidth() / 2);
    if (!brush.IsHollow()) {
      vertices.BindInnerCircle(vp);
      brush.Set();
      glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.CIRCLE_SIZE);
    }
    vertices.Bind(vp);
    pen.Bind();
    glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices.SIZE);
    pen.Unbind();
  } else if (OpenGL::vertex_buffer_object && radius < 16) {
    /* draw a "small" circle with VBO */

    OpenGL::small_circle_buffer->Bind();
    const ScopeVertexPointer vp(nullptr);

#ifdef HAVE_GLES2
    glm::mat4 matrix2 = glm::scale(glm::translate(glm::mat4(),
                                                  glm::vec3(x, y, 0)),
                                   glm::vec3(radius / 256.));
    glUniformMatrix4fv(OpenGL::solid_modelview, 1, GL_FALSE,
                       glm::value_ptr(matrix2));
#else
    glPushMatrix();

#ifdef HAVE_GLES
    glTranslatex((GLfixed)x << 16, (GLfixed)y << 16, 0);
    glScalex((GLfixed)radius << 8, (GLfixed)radius << 8, (GLfixed)1 << 16);
#else
    glTranslatef(x, y, 0.);
    glScalef(radius / 256., radius / 256., 1.);
#endif
#endif

    if (!brush.IsHollow()) {
      brush.Set();
      glDrawArrays(GL_TRIANGLE_FAN, 0, OpenGL::SMALL_CIRCLE_SIZE);
    }

    if (IsPenOverBrush()) {
      pen.Bind();
      glDrawArrays(GL_LINE_LOOP, 0, OpenGL::SMALL_CIRCLE_SIZE);
      pen.Unbind();
    }

#ifdef HAVE_GLES2
    glUniformMatrix4fv(OpenGL::solid_modelview, 1, GL_FALSE,
                       glm::value_ptr(glm::mat4()));
#else
    glPopMatrix();
#endif

    OpenGL::small_circle_buffer->Unbind();
#ifdef HAVE_GLES2
  } else {
#else
  } else if (OpenGL::vertex_buffer_object) {
#endif
    /* draw a "big" circle with VBO */

    OpenGL::circle_buffer->Bind();
    const ScopeVertexPointer vp(nullptr);

#ifdef HAVE_GLES2
    glm::mat4 matrix2 = glm::scale(glm::translate(glm::mat4(),
                                                  glm::vec3(x, y, 0)),
                                   glm::vec3(radius / 1024.));
    glUniformMatrix4fv(OpenGL::solid_modelview, 1, GL_FALSE,
                       glm::value_ptr(matrix2));
#else
    glPushMatrix();

#ifdef HAVE_GLES
    glTranslatex((GLfixed)x << 16, (GLfixed)y << 16, 0);
    glScalex((GLfixed)radius << 6, (GLfixed)radius << 6, (GLfixed)1 << 16);
#else
    glTranslatef(x, y, 0.);
    glScalef(radius / 1024., radius / 1024., 1.);
#endif
#endif

    if (!brush.IsHollow()) {
      brush.Set();
      glDrawArrays(GL_TRIANGLE_FAN, 0, OpenGL::CIRCLE_SIZE);
    }

    if (IsPenOverBrush()) {
      pen.Bind();
      glDrawArrays(GL_LINE_LOOP, 0, OpenGL::CIRCLE_SIZE);
      pen.Unbind();
    }

#ifdef HAVE_GLES2
    glUniformMatrix4fv(OpenGL::solid_modelview, 1, GL_FALSE,
                       glm::value_ptr(glm::mat4()));
#else
    glPopMatrix();
#endif

    OpenGL::circle_buffer->Unbind();
#ifndef HAVE_GLES2
  } else {
    ScopeVertexPointer vp;
    GLCircleVertices vertices(x, y, radius);
    vertices.Bind(vp);

    if (!brush.IsHollow()) {
      brush.Set();
      glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.SIZE);
    }

    if (IsPenOverBrush()) {
      pen.Bind();
      glDrawArrays(GL_LINE_LOOP, 0, vertices.SIZE);
      pen.Unbind();
    }
#endif
  }
}

void
Canvas::DrawSegment(int x, int y, unsigned radius,
                    Angle start, Angle end, bool horizon)
{
  ::Segment(*this, x, y, radius, start, end, horizon);
}

gcc_const
static unsigned
AngleToDonutVertex(Angle angle)
{
  return GLDonutVertices::ImportAngle(NATIVE_TO_INT(angle.Native())
                                      + ARRAY_SIZE(ISINETABLE) * 3u / 4u,
                                      ARRAY_SIZE(ISINETABLE));
}

gcc_const
static std::pair<unsigned,unsigned>
AngleToDonutVertices(Angle start, Angle end)
{
  static constexpr Angle epsilon = Angle::FullCircle()
    / int(GLDonutVertices::CIRCLE_SIZE * 4u);

  const Angle delta = end - start;

  if (fabs(delta.AsDelta().Native()) <= epsilon.Native())
    /* full circle */
    return std::make_pair(0u, unsigned(GLDonutVertices::MAX_ANGLE));

  const unsigned istart = AngleToDonutVertex(start);
  unsigned iend = AngleToDonutVertex(end);

  if (istart == iend && delta > epsilon) {
    if (end - start >= Angle::HalfCircle())
      /* nearly full circle, round down the end */
      iend = GLDonutVertices::PreviousAngle(iend);
    else
      /* slightly larger than epsilon: draw at least two indices */
      iend = GLDonutVertices::NextAngle(iend);
  }

  return std::make_pair(istart, iend);
}

void
Canvas::DrawAnnulus(int x, int y,
                    unsigned small_radius, unsigned big_radius,
                    Angle start, Angle end)
{
  if (1 == 1) {
    /* TODO: switched to the unoptimised generic implementation due to
       TRAC #2221, caused by rounding error of start/end radial;
       should reimplement GLDonutVertices to use the exact start/end
       radial */
    ::Annulus(*this, x, y, big_radius, start, end, small_radius);
    return;
  }

  ScopeVertexPointer vp;
  GLDonutVertices vertices(x, y, small_radius, big_radius);

  const std::pair<unsigned,unsigned> i = AngleToDonutVertices(start, end);
  const unsigned istart = i.first;
  const unsigned iend = i.second;

  if (!brush.IsHollow()) {
    brush.Set();
    vertices.Bind(vp);

    if (istart > iend) {
      glDrawArrays(GL_TRIANGLE_STRIP, istart,
                   GLDonutVertices::MAX_ANGLE - istart + 2);
      glDrawArrays(GL_TRIANGLE_STRIP, 0, iend + 2);
    } else {
      glDrawArrays(GL_TRIANGLE_STRIP, istart, iend - istart + 2);
    }
  }

  if (IsPenOverBrush()) {
    pen.Bind();

    if (istart != iend && iend != GLDonutVertices::MAX_ANGLE) {
      if (brush.IsHollow())
        vertices.Bind(vp);

      glDrawArrays(GL_LINE_STRIP, istart, 2);
      glDrawArrays(GL_LINE_STRIP, iend, 2);
    }

    const unsigned pstart = istart / 2;
    const unsigned pend = iend / 2;

    vertices.BindInnerCircle(vp);
    if (pstart < pend) {
      glDrawArrays(GL_LINE_STRIP, pstart, pend - pstart + 1);
    } else {
      glDrawArrays(GL_LINE_STRIP, pstart,
                   GLDonutVertices::CIRCLE_SIZE - pstart + 1);
      glDrawArrays(GL_LINE_STRIP, 0, pend + 1);
    }

    vertices.BindOuterCircle(vp);
    if (pstart < pend) {
      glDrawArrays(GL_LINE_STRIP, pstart, pend - pstart + 1);
    } else {
      glDrawArrays(GL_LINE_STRIP, pstart,
                   GLDonutVertices::CIRCLE_SIZE - pstart + 1);
      glDrawArrays(GL_LINE_STRIP, 0, pend + 1);
    }

    pen.Unbind();
  }
}

void
Canvas::DrawKeyhole(int x, int y,
                    unsigned small_radius, unsigned big_radius,
                    Angle start, Angle end)
{
  ::KeyHole(*this, x, y, big_radius, start, end, small_radius);
}

void
Canvas::DrawFocusRectangle(PixelRect rc)
{
  DrawOutlineRectangle(rc.left, rc.top, rc.right, rc.bottom, COLOR_DARK_GRAY);
}

const PixelSize
Canvas::CalcTextSize(const TCHAR *text) const
{
  assert(text != nullptr);
#ifndef UNICODE
  assert(ValidateUTF8(text));
#endif

  PixelSize size = { 0, 0 };

  if (font == nullptr)
    return size;

  /* see if the TextCache can handle this request */
  size = TextCache::LookupSize(*font, text);
  if (size.cy > 0)
    return size;

  return TextCache::GetSize(*font, text);
}

/**
 * Prepare drawing a GL_ALPHA texture with the specified color.
 */
static void
PrepareColoredAlphaTexture(Color color)
{
#ifdef HAVE_GLES2
  OpenGL::alpha_shader->Use();
  color.Set();
#else
  color.Set();

  if (color == COLOR_BLACK) {
    /* GL_ALPHA textures have black RGB - this is easy */

    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  } else {
    /* use GL_COMBINE to replace the texture color (black) with the
       specified one */
    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);

    /* replace the texture color with the selected text color */
    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE);
    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_SRC0_RGB, GL_PREVIOUS);
    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_SRC_COLOR);

    /* use the texture alpha */
    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_ALPHA, GL_REPLACE);
    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_SRC0_ALPHA, GL_TEXTURE);
    OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_ALPHA, GL_SRC_ALPHA);
  }
#endif
}

void
Canvas::DrawText(int x, int y, const TCHAR *text)
{
  assert(text != nullptr);
  assert(ValidateUTF8(text));

#ifdef HAVE_GLES
  assert(offset == OpenGL::translate);
#endif

  if (font == nullptr)
    return;

  GLTexture *texture = TextCache::Get(*font, text);
  if (texture == nullptr)
    return;

  if (background_mode == OPAQUE)
    DrawFilledRectangle(x, y,
                        x + texture->GetWidth(), y + texture->GetHeight(),
                        background_color);

  PrepareColoredAlphaTexture(text_color);

#ifndef HAVE_GLES2
  GLEnable scope(GL_TEXTURE_2D);
#endif

  const GLBlend blend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  texture->Bind();
  texture->Draw(x, y);
}

void
Canvas::DrawTransparentText(int x, int y, const TCHAR *text)
{
  assert(text != nullptr);
  assert(ValidateUTF8(text));

#ifdef HAVE_GLES
  assert(offset == OpenGL::translate);
#endif

  if (font == nullptr)
    return;

  GLTexture *texture = TextCache::Get(*font, text);
  if (texture == nullptr)
    return;

  PrepareColoredAlphaTexture(text_color);

#ifndef HAVE_GLES2
  GLEnable scope(GL_TEXTURE_2D);
#endif

  const GLBlend blend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  texture->Bind();
  texture->Draw(x, y);
}

void
Canvas::DrawClippedText(int x, int y,
                        unsigned width, unsigned height,
                        const TCHAR *text)
{
  assert(text != nullptr);
  assert(ValidateUTF8(text));

#ifdef HAVE_GLES
  assert(offset == OpenGL::translate);
#endif

  if (font == nullptr)
    return;

  GLTexture *texture = TextCache::Get(*font, text);
  if (texture == nullptr)
    return;

  if (texture->GetHeight() < height)
    height = texture->GetHeight();
  if (texture->GetWidth() < width)
    width = texture->GetWidth();

  PrepareColoredAlphaTexture(text_color);

#ifndef HAVE_GLES2
  GLEnable scope(GL_TEXTURE_2D);
#endif

  const GLBlend blend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  texture->Bind();
  texture->Draw(x, y, width, height, 0, 0, width, height);
}

void
Canvas::Stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const GLTexture &texture,
                int src_x, int src_y,
                unsigned src_width, unsigned src_height)
{
#ifdef HAVE_GLES
  assert(offset == OpenGL::translate);
#endif

#ifdef HAVE_GLES2
  OpenGL::texture_shader->Use();
#endif

  texture.Draw(dest_x, dest_y, dest_width, dest_height,
               src_x, src_y, src_width, src_height);
}

void
Canvas::Stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const GLTexture &texture)
{
  Stretch(dest_x, dest_y, dest_width, dest_height,
          texture, 0, 0, texture.GetWidth(), texture.GetHeight());
}

void
Canvas::Copy(int dest_x, int dest_y,
             unsigned dest_width, unsigned dest_height,
             const Bitmap &src, int src_x, int src_y)
{
  Stretch(dest_x, dest_y, dest_width, dest_height,
          src, src_x, src_y, dest_width, dest_height);
}

void
Canvas::Copy(const Bitmap &src)
{
  Copy(0, 0, src.GetWidth(), src.GetHeight(), src, 0, 0);
}

void
Canvas::StretchNot(const Bitmap &src)
{
  assert(src.IsDefined());

#ifdef HAVE_GLES2
  OpenGL::invert_shader->Use();
#else
  const GLEnable scope(GL_TEXTURE_2D);

  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);

  /* invert the texture color */
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_SRC0_RGB, GL_TEXTURE);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_ONE_MINUS_SRC_COLOR);

  /* copy the texture alpha */
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_ALPHA, GL_REPLACE);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_SRC0_ALPHA, GL_TEXTURE);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_ALPHA, GL_SRC_ALPHA);
#endif

  GLTexture &texture = *src.GetNative();
  texture.Draw(0, 0, GetWidth(), GetHeight(),
               0, 0, src.GetWidth(), src.GetHeight());
}

void
Canvas::Stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const Bitmap &src, int src_x, int src_y,
                unsigned src_width, unsigned src_height)
{
#ifdef HAVE_GLES
  assert(offset == OpenGL::translate);
#endif
  assert(src.IsDefined());

#ifdef HAVE_GLES2
  OpenGL::texture_shader->Use();
#else
  const GLEnable scope(GL_TEXTURE_2D);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
#endif

  GLTexture &texture = *src.GetNative();
  texture.Bind();
  texture.Draw(dest_x, dest_y, dest_width, dest_height,
               src_x, src_y, src_width, src_height);
}

void
Canvas::Stretch(int dest_x, int dest_y,
                unsigned dest_width, unsigned dest_height,
                const Bitmap &src)
{
#ifdef HAVE_GLES
  assert(offset == OpenGL::translate);
#endif
  assert(src.IsDefined());

#ifdef HAVE_GLES2
  OpenGL::texture_shader->Use();
#else
  const GLEnable scope(GL_TEXTURE_2D);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
#endif

  GLTexture &texture = *src.GetNative();
  texture.Bind();
  texture.Draw(dest_x, dest_y, dest_width, dest_height,
               0, 0, src.GetWidth(), src.GetHeight());
}

void
Canvas::StretchMono(int dest_x, int dest_y,
                    unsigned dest_width, unsigned dest_height,
                    const Bitmap &src,
                    int src_x, int src_y,
                    unsigned src_width, unsigned src_height,
                    Color fg_color, Color bg_color)
{
  /* note that this implementation ignores the background color; it is
     not mandatory, and we can assume that the background is already
     set; it is only being passed to this function because the GDI
     implementation will be faster when erasing the background
     again */

#ifdef HAVE_GLES2
  OpenGL::alpha_shader->Use();
  fg_color.Set();
#else
  const GLEnable scope(GL_TEXTURE_2D);

  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);

  /* replace the texture color with the selected text color */
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_SRC0_RGB, GL_PREVIOUS);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_SRC_COLOR);

  /* invert texture alpha (our bitmaps have black text on white
     background) */
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_ALPHA, GL_REPLACE);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_SRC0_ALPHA, GL_TEXTURE);
  OpenGL::glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#endif

  const GLBlend blend(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  GLTexture &texture = *src.GetNative();
  texture.Bind();
  texture.Draw(dest_x, dest_y, dest_width, dest_height,
               src_x, src_y, src_width, src_height);
}

void
Canvas::CopyToTexture(GLTexture &texture, PixelRect src_rc) const
{
#ifdef HAVE_GLES
  assert(offset == OpenGL::translate);
#endif

  texture.Bind();
  glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                      OpenGL::translate.x + src_rc.left,
                      OpenGL::viewport_size.y - OpenGL::translate.y - src_rc.bottom,
                      src_rc.right - src_rc.left,
                      src_rc.bottom - src_rc.top);

}

void
Canvas::DrawRoundRectangle(int left, int top, int right, int bottom,
                           unsigned ellipse_width,
                           unsigned ellipse_height)
{
  unsigned radius = std::min(std::min(ellipse_width, ellipse_height),
                             unsigned(std::min(bottom - top,
                                               right - left))) / 2u;
  ::RoundRect(*this, left, top, right, bottom, radius);
}
