int
pcl::io::savePLYFile (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision)
{
  if (mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());
  if (!fs)
  {
    PCL_ERROR ("[pcl::io::savePLYFile] Error during opening (%s)!\n", file_name.c_str ());
    return (-1);
  }

  // number of points
  size_t nr_points  = mesh.cloud.width * mesh.cloud.height;
  size_t point_size = mesh.cloud.data.size () / nr_points;

  // number of faces
  size_t nr_faces = mesh.polygons.size ();

  // Write header
  fs << "ply";
  fs << "\nformat ascii 1.0";
  fs << "\ncomment PCL generated";
  // Vertices
  fs << "\nelement vertex "<< mesh.cloud.width * mesh.cloud.height;
  fs << "\nproperty float x"
        "\nproperty float y"
        "\nproperty float z";
  // Check if we have color on vertices
  int rgba_index = getFieldIndex (mesh.cloud, "rgba"),
  rgb_index = getFieldIndex (mesh.cloud, "rgb");
  if (rgba_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue"
          "\nproperty uchar alpha";
  }
  else if (rgb_index != -1)
  {
    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue";
  }
  // Check if we have normal on vertices
  int normal_x_index = getFieldIndex(mesh.cloud, "normal_x");
  int normal_y_index = getFieldIndex(mesh.cloud, "normal_y");
  int normal_z_index = getFieldIndex(mesh.cloud, "normal_z");
  if (normal_x_index != -1 && normal_y_index != -1 && normal_z_index != -1)
  {
      fs << "\nproperty float nx"
            "\nproperty float ny"
            "\nproperty float nz";
  }
  // Check if we have curvature on vertices
  int curvature_index = getFieldIndex(mesh.cloud, "curvature");
  if ( curvature_index != -1)
  {
      fs << "\nproperty float curvature";
  }
  // Faces
  fs << "\nelement face "<< nr_faces;
  fs << "\nproperty list uchar int vertex_indices";
  fs << "\nend_header\n";

  // Write down vertices
  for (size_t i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      int count = mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;

      // adding vertex
      if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          mesh.cloud.fields[d].name == "x" ||
          mesh.cloud.fields[d].name == "y" ||
          mesh.cloud.fields[d].name == "z"))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        // if (++xyz == 3)
        //   break;
        ++xyz;
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) &&
                (mesh.cloud.fields[d].name == "rgb"))

      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgb_index].offset + c * sizeof (float)], sizeof (RGB));
        fs << int (color.r) << " " << int (color.g) << " " << int (color.b);
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::UINT32) &&
               (mesh.cloud.fields[d].name == "rgba"))
      {
        pcl::RGB color;
        memcpy (&color, &mesh.cloud.data[i * point_size + mesh.cloud.fields[rgba_index].offset + c * sizeof (uint32_t)], sizeof (RGB));
        fs << int (color.r) << " " << int (color.g) << " " << int (color.b) << " " << int (color.a);
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                mesh.cloud.fields[d].name == "normal_x" ||
                mesh.cloud.fields[d].name == "normal_y" ||
                mesh.cloud.fields[d].name == "normal_z"))
      {
        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof(float)], sizeof(float));
        fs << value;
      }
      else if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                mesh.cloud.fields[d].name == "curvature"))
      {
        float value;
        memcpy(&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof(float)], sizeof(float));
        fs << value;
      }
      fs << " ";
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::savePLYFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << '\n';
  }

  // Write down faces
  for (size_t i = 0; i < nr_faces; i++)
  {
    fs << mesh.polygons[i].vertices.size () << " ";
    size_t j = 0;
    for (j = 0; j < mesh.polygons[i].vertices.size () - 1; ++j)
      fs << mesh.polygons[i].vertices[j] << " ";
    fs << mesh.polygons[i].vertices[j] << '\n';
  }

  // Close file
  fs.close ();
  return (0);
}

int
pcl::PLYReader::read (const std::string &file_name, pcl::PolygonMesh &mesh,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                      int &ply_version, const int offset)
{
  // kept only for backward compatibility
  int data_type;
  unsigned int data_idx;
  polygons_ = &(mesh.polygons);
  if (this->readHeader (file_name, mesh.cloud, origin, orientation, ply_version, data_type, data_idx, offset))
  {
    PCL_ERROR ("[pcl::PLYReader::read] problem parsing header!\n");
    return (-1);
  }

  // a range_grid element was found ?
  size_t r_size;
  if ((r_size  = (*range_grid_).size ()) > 0 && r_size != vertex_count_)
  {
    //cloud.header = cloud_->header;
    std::vector<pcl::uint8_t> data ((*range_grid_).size () * mesh.cloud.point_step);
    const static float f_nan = std::numeric_limits <float>::quiet_NaN ();
    const static double d_nan = std::numeric_limits <double>::quiet_NaN ();
    for (size_t r = 0; r < r_size; ++r)
    {
      if ((*range_grid_)[r].size () == 0)
      {
        for (size_t f = 0; f < cloud_->fields.size (); ++f)
          if (cloud_->fields[f].datatype == ::pcl::PCLPointField::FLOAT32)
            memcpy (&data[r * cloud_->point_step + cloud_->fields[f].offset],
                    reinterpret_cast<const char*> (&f_nan), sizeof (float));
          else if (cloud_->fields[f].datatype == ::pcl::PCLPointField::FLOAT64)
            memcpy (&data[r * cloud_->point_step + cloud_->fields[f].offset],
                    reinterpret_cast<const char*> (&d_nan), sizeof (double));
          else
            memset (&data[r * cloud_->point_step + cloud_->fields[f].offset], 0,
                    pcl::getFieldSize (cloud_->fields[f].datatype) * cloud_->fields[f].count);
      }
      else
        memcpy (&data[r* cloud_->point_step], &cloud_->data[(*range_grid_)[r][0] * cloud_->point_step], cloud_->point_step);
    }
    cloud_->data.swap (data);
  }

  orientation_ = Eigen::Quaternionf (orientation);
  origin_ = origin;

  for (size_t i = 0; i < cloud_->fields.size (); ++i)
  {
    if (cloud_->fields[i].name == "nx")
      cloud_->fields[i].name = "normal_x";
    if (cloud_->fields[i].name == "ny")
      cloud_->fields[i].name = "normal_y";
    if (cloud_->fields[i].name == "nz")
      cloud_->fields[i].name = "normal_z";
  }
  return (0);
}

int
pcl::PLYReader::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &, int &, unsigned int &, const int)
{
  // Silence compiler warnings
  cloud_ = &cloud;
  range_grid_ = new std::vector<std::vector<int> >;
  cloud_->width = cloud_->height = 0;
  origin = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  if (!parse (file_name))
  {
    PCL_ERROR ("[pcl::PLYReader::read] problem parsing header!\n");
    return (-1);
  }
  cloud_->row_step = cloud_->point_step * cloud_->width;
  return 0;
}
