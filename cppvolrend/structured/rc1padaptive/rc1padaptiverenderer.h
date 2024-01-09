/**
 * Isosurface raycaster.
 *
 * @author Tino Weinkauf
**/
#pragma once

#include "../../volrenderbase.h"

class RayCastingAdaptiveIso : public BaseVolumeRenderer
{
public:
  RayCastingAdaptiveIso();
  virtual ~RayCastingAdaptiveIso();
  
  virtual const char* GetName();
  virtual const char* GetAbbreviationName();
  virtual vis::GRID_VOLUME_DATA_TYPE GetDataTypeSupport();

  virtual void Clean();
  virtual bool Init(int shader_width, int shader_height);
  virtual void ReloadShaders();

  virtual bool Update(vis::Camera* camera);
  virtual void Redraw();

  virtual void FillParameterSpace(ParameterSpace& pspace) override;
  virtual void Reshape(int w, int h) override;

  virtual void SetImGuiComponents();

protected:
  float m_u_isovalue;
  float m_u_step_size; // quality of isosurface 
  glm::vec4 m_u_color; // RGBA float
  bool m_apply_gradient_shading;

private:
  gl::ComputeShader* cp_shader_rendering;
  bool m_do_interpolation;
  bool m_debug_temp_texture;
  bool m_visualize_pixel_diff;
  bool m_use_luminance_diff;
  int m_interval;
  float m_max_pixel_difference;
  gl::Texture2D *m_temp_texture;
};

