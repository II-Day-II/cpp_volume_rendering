#include "../../defines.h"
#include "rc1padaptiverenderer.h"

#include <vis_utils/camera.h>
#include <volvis_utils/datamanager.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <math_utils/utils.h>


RayCastingAdaptiveIso::RayCastingAdaptiveIso()
    :cp_shader_rendering(nullptr)
    , m_u_isovalue(0.5f)
    , m_u_step_size(0.5f)
    , m_u_color(0.66f, 0.6f, 0.05f, 1.0f)
    , m_apply_gradient_shading(false)
    , m_do_interpolation(false)
    , m_interval(1)
    , m_temp_texture(nullptr) // bruh
    , m_debug_temp_texture(false)
{
}


RayCastingAdaptiveIso::~RayCastingAdaptiveIso()
{
}


const char* RayCastingAdaptiveIso::GetName()
{
  return "Adaptive resolution - Isosurface Raycaster";
}


const char* RayCastingAdaptiveIso::GetAbbreviationName()
{
  return "Adaptive";
}


vis::GRID_VOLUME_DATA_TYPE RayCastingAdaptiveIso::GetDataTypeSupport()
{
  return vis::GRID_VOLUME_DATA_TYPE::STRUCTURED;
}


void RayCastingAdaptiveIso::Clean()
{
  if (cp_shader_rendering) delete cp_shader_rendering;
  cp_shader_rendering = nullptr;

  gl::ExitOnGLError("Could not destroy shaders");

  if (m_temp_texture) delete m_temp_texture;
  m_temp_texture = nullptr;

  BaseVolumeRenderer::Clean();
}

void RayCastingAdaptiveIso::Reshape(int w, int h) // override
{
    // reshape temp texture as m_rdr_...'s texture will be reshaped
    if (m_temp_texture == nullptr)
    {
        m_temp_texture = new gl::Texture2D(w, h);
        m_temp_texture->GenerateTexture(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE);
        m_temp_texture->SetData(NULL, GL_RGBA16F, GL_RGBA, GL_FLOAT);
    }
    else
    {
        if (m_temp_texture->GetWidth() != w || m_temp_texture->GetHeight() != h)
        {
            delete m_temp_texture;

            m_temp_texture = new gl::Texture2D(w, h);
            m_temp_texture->GenerateTexture(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE);
            m_temp_texture->SetData(NULL, GL_RGBA16F, GL_RGBA, GL_FLOAT);
        }
    }
    gl::ExitOnGLError("RenderFrameToScreen: Error on UpdateScreenResolution.");
    // also do the normal reshaping
    BaseVolumeRenderer::Reshape(w, h);
}


bool RayCastingAdaptiveIso::Init(int swidth, int sheight)
{
  //Clean before we continue
  if (IsBuilt()) Clean();

  //We need data to work on
  if (m_ext_data_manager->GetCurrentVolumeTexture() == nullptr) return false;


  ////////////////////////////////////////////
  // Create Rendering Buffers and Shaders

  // - generate temp texture
  //temp.GenerateTexture(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE); // use nearest since it's the same resolution, no filtering should apply
  //temp.SetData(NULL, GL_RGBA16F, GL_RGBA, GL_FLOAT); // make sure format is correct or smth

  // - definition of uniform grid and bounding box
  glm::vec3 vol_resolution = glm::vec3(m_ext_data_manager->GetCurrentStructuredVolume()->GetWidth() ,
                                       m_ext_data_manager->GetCurrentStructuredVolume()->GetHeight(),
                                       m_ext_data_manager->GetCurrentStructuredVolume()->GetDepth() );

  glm::vec3 vol_voxelsize = glm::vec3(m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleX(),
                                      m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleY(),
                                      m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleZ());

  glm::vec3 vol_aabb = vol_resolution * vol_voxelsize;

  // - load shaders
  cp_shader_rendering = new gl::ComputeShader();
  cp_shader_rendering->AddShaderFile(CPPVOLREND_DIR"structured/_common_shaders/ray_bbox_intersection.comp");
  cp_shader_rendering->AddShaderFile(CPPVOLREND_DIR"structured/rc1padaptive/ray_marching_1p_iso_adaptive.comp");
  cp_shader_rendering->LoadAndLink();
  cp_shader_rendering->Bind();

  // - data sets to work on: scalar field and its gradient
  if (m_ext_data_manager->GetCurrentVolumeTexture())
    cp_shader_rendering->SetUniformTexture3D("TexVolume", m_ext_data_manager->GetCurrentVolumeTexture()->GetTextureID(), 1);
  if (m_apply_gradient_shading && m_ext_data_manager->GetCurrentGradientTexture())
    cp_shader_rendering->SetUniformTexture3D("TexVolumeGradient", m_ext_data_manager->GetCurrentGradientTexture()->GetTextureID(), 2);

  // - let the shader know about the uniform grid
  cp_shader_rendering->SetUniform("VolumeGridResolution", vol_resolution);
  cp_shader_rendering->SetUniform("VolumeVoxelSize", vol_voxelsize);
  cp_shader_rendering->SetUniform("VolumeGridSize", vol_aabb);

  cp_shader_rendering->BindUniforms();
  cp_shader_rendering->Unbind();
  gl::ExitOnGLError("RayCastingAdaptiveIso: Error on Preparing Models and Shaders");


  /////////////////////////////////
  // Finalization

  //Support for multisampling
  Reshape(swidth, sheight);

  SetBuilt(true);
  SetOutdated();
  return true;
}


void RayCastingAdaptiveIso::ReloadShaders()
{
  cp_shader_rendering->Reload();
  m_rdr_frame_to_screen.ClearShaders();
}


bool RayCastingAdaptiveIso::Update(vis::Camera* camera)
{
  cp_shader_rendering->Bind();

  /////////////////////////////
  // Multisample
  if (IsPixelMultiScalingSupported() && GetCurrentMultiScalingMode() > 0)
  {
    cp_shader_rendering->RecomputeNumberOfGroups(m_rdr_frame_to_screen.GetWidth(),
                                                 m_rdr_frame_to_screen.GetHeight(), 0);
  }
  else
  {
    cp_shader_rendering->RecomputeNumberOfGroups(m_ext_rendering_parameters->GetScreenWidth(),
                                                 m_ext_rendering_parameters->GetScreenHeight(), 0);
  }

  /////////////////////////////
  // Camera
  cp_shader_rendering->SetUniform("CameraEye", camera->GetEye());
  cp_shader_rendering->BindUniform("CameraEye");

  cp_shader_rendering->SetUniform("u_CameraLookAt", camera->LookAt());
  cp_shader_rendering->BindUniform("u_CameraLookAt");

  cp_shader_rendering->SetUniform("ProjectionMatrix", camera->Projection());
  cp_shader_rendering->BindUniform("ProjectionMatrix");

  cp_shader_rendering->SetUniform("u_TanCameraFovY", (float)tan(DEGREE_TO_RADIANS(camera->GetFovY()) / 2.0));
  cp_shader_rendering->BindUniform("u_TanCameraFovY");

  cp_shader_rendering->SetUniform("u_CameraAspectRatio", camera->GetAspectRatio());
  cp_shader_rendering->BindUniform("u_CameraAspectRatio");

  cp_shader_rendering->SetUniform("WorldEyePos", camera->GetEye());
  cp_shader_rendering->BindUniform("WorldEyePos");

  /////////////////////////////
  // Isosurface aspects
  cp_shader_rendering->SetUniform("Isovalue", m_u_isovalue);
  cp_shader_rendering->SetUniform("StepSize", m_u_step_size);
  //cp_shader_rendering->BindUniform("StepSize");
  cp_shader_rendering->SetUniform("Color", m_u_color);
  //cp_shader_rendering->BindUniform("Color");

  /////////////////////////////
  // Adaptive settings
  cp_shader_rendering->SetUniform("DoInterpolation", m_do_interpolation);
  cp_shader_rendering->SetUniform("interval", m_interval);


  cp_shader_rendering->SetUniform("ApplyGradientPhongShading", (m_apply_gradient_shading && m_ext_data_manager->GetCurrentGradientTexture()) ? 1 : 0);
  cp_shader_rendering->BindUniform("ApplyGradientPhongShading");

  cp_shader_rendering->SetUniform("BlinnPhongKa", m_ext_rendering_parameters->GetBlinnPhongKambient());
  cp_shader_rendering->BindUniform("BlinnPhongKa");
  cp_shader_rendering->SetUniform("BlinnPhongKd", m_ext_rendering_parameters->GetBlinnPhongKdiffuse());
  cp_shader_rendering->BindUniform("BlinnPhongKd");
  cp_shader_rendering->SetUniform("BlinnPhongKs", m_ext_rendering_parameters->GetBlinnPhongKspecular());
  cp_shader_rendering->BindUniform("BlinnPhongKs");
  cp_shader_rendering->SetUniform("BlinnPhongShininess", m_ext_rendering_parameters->GetBlinnPhongNshininess());
  cp_shader_rendering->BindUniform("BlinnPhongShininess");

  cp_shader_rendering->SetUniform("BlinnPhongIspecular", m_ext_rendering_parameters->GetLightSourceSpecular());
  cp_shader_rendering->BindUniform("BlinnPhongIspecular");

  cp_shader_rendering->SetUniform("LightSourcePosition", m_ext_rendering_parameters->GetBlinnPhongLightingPosition());
  cp_shader_rendering->BindUniform("LightSourcePosition");

  // calls binduniform on all uniforms that have been set
  cp_shader_rendering->BindUniforms();

  gl::Shader::Unbind();
  gl::ExitOnGLError("RayCastingAdaptiveIso: After Update.");
  return true;
}


void RayCastingAdaptiveIso::Redraw()
{
    if (m_do_interpolation)
    {
        // FIRST PASS
        // clear the temp texture
        glClearTexImage(m_temp_texture->GetTextureID(), 0, GL_RGBA, GL_FLOAT, 0);

        cp_shader_rendering->Bind();
        cp_shader_rendering->SetUniform("IsFirstPass", true);
        cp_shader_rendering->BindUniform("IsFirstPass"); // idk if this actually does anything

        // bind the temp texture as output ( this is what rdr_frame_to_screen.BindImageTexture does )
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_temp_texture->GetTextureID());
        glBindImageTexture(0, m_temp_texture->GetTextureID(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);

        // render to the temp texture
        cp_shader_rendering->Dispatch();
        gl::ComputeShader::Unbind();

        // END FIRST PASS

        if (m_debug_temp_texture)
        {
            m_rdr_frame_to_screen.Draw(m_temp_texture);
        } 
        else
        {
            // SECOND PASS
            m_rdr_frame_to_screen.ClearTexture();
            cp_shader_rendering->Bind();
            m_rdr_frame_to_screen.BindImageTexture();

            cp_shader_rendering->SetUniform("IsFirstPass", false);
            cp_shader_rendering->BindUniform("IsFirstPass");


            cp_shader_rendering->SetUniform("tempTexture", 3); // wtf
            cp_shader_rendering->BindUniform("tempTexture");
            glActiveTexture(GL_TEXTURE3); // aaaaaah
            glBindTexture(GL_TEXTURE_2D, m_temp_texture->GetTextureID()); // this feels wrong. Very wrong.


            cp_shader_rendering->Dispatch();
            gl::ComputeShader::Unbind();

            m_rdr_frame_to_screen.Draw();
        }
        // END SECOND PASS
    }
    else 
    {
        m_rdr_frame_to_screen.ClearTexture();
  
        cp_shader_rendering->Bind();
        m_rdr_frame_to_screen.BindImageTexture();
  
        cp_shader_rendering->Dispatch();
        gl::ComputeShader::Unbind();
 
        m_rdr_frame_to_screen.Draw();
    }
}


void RayCastingAdaptiveIso::FillParameterSpace(ParameterSpace& pspace) 
{
    pspace.ClearParameterDimensions();
    pspace.AddParameterDimension(new ParameterRangeFloat("StepSize", &m_u_step_size, 0.05f, 3.0f, 0.1f));
    // add more parameters to evaluate
}

void RayCastingAdaptiveIso::SetImGuiComponents()
{
  ImGui::Separator();
  
  ImGui::Text("Isovalue: ");
  if (ImGui::DragFloat("###RayCastingAdaptiveIsoUIIsovalue", &m_u_isovalue, 0.01f, 0.01f, 100.0f, "%.2f"))
  {
    m_u_isovalue = std::max(std::min(m_u_isovalue, 100.0f), 0.01f); //When entering with keyboard, ImGui does not take care of the min/max.
    SetOutdated();
  }
  
  ImGui::Text("StepSize: ");
  if (ImGui::DragFloat("###RayCastingAdaptiveIsoUIStepSize", &m_u_step_size, 0.01f, 0.05f, 3.0f, "%.2f"))
  {
      m_u_step_size = std::max(std::min(m_u_step_size, 3.0f), 0.05f); //When entering with keyboard, ImGui does not take care of the min/max.
      SetOutdated();
  }

  ImGui::Text("Do interpolation: ");
  if (ImGui::Checkbox("###RayCastingAdaptiveIsoUIDoInterpolation", &m_do_interpolation))
  {
      SetOutdated();
  }
  
  ImGui::Text("Debug temp texture");
  if (ImGui::Checkbox("###RayCastingAdaptiveIsoUIDebug", &m_debug_temp_texture))
  {
      SetOutdated();
  }

  ImGui::Text("Raycasting pixel Interval: ");
  if (ImGui::DragInt("###RayCastingAdaptiveIsoUIPixelInterval", &m_interval, 1, 1, 16)) // is there a PowerSlider?
  {
      m_interval = std::max(std::min(m_interval, 16), 1);
      SetOutdated();
  }
  
  if (ImGui::ColorEdit4("Color", &m_u_color[0]))
  {
      SetOutdated();
  }
  //AddImGuiMultiSampleOptions();
  
  if (m_ext_data_manager->GetCurrentGradientTexture())
  {
    ImGui::Separator();
    if (ImGui::Checkbox("Apply Gradient Shading", &m_apply_gradient_shading))
    {
      // Delete current uniform
      cp_shader_rendering->ClearUniform("TexVolumeGradient");

      if (m_apply_gradient_shading && m_ext_data_manager->GetCurrentGradientTexture())
      {
        cp_shader_rendering->Bind();
        cp_shader_rendering->SetUniformTexture3D("TexVolumeGradient", m_ext_data_manager->GetCurrentGradientTexture()->GetTextureID(), 2);
        cp_shader_rendering->BindUniform("TexVolumeGradient");
        gl::ComputeShader::Unbind();
      }
      SetOutdated();
    }
    ImGui::Separator();
  }
}
