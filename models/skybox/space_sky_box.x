xof 0303txt 0032


template VertexDuplicationIndices { 
  <b8d65549-d7c9-4995-89cf-53a9a8b031e3>
  DWORD nIndices;
  DWORD nOriginalVertices;
  array DWORD indices[nIndices];
 }
 template XSkinMeshHeader {
  <3cf169ce-ff7c-44ab-93c0-f78f62d172e2>
  WORD nMaxSkinWeightsPerVertex;
  WORD nMaxSkinWeightsPerFace;
  WORD nBones;
 }
 template SkinWeights {
  <6f0d123b-bad2-4167-a0d0-80224f25fabb>
  STRING transformNodeName;
  DWORD nWeights;
  array DWORD vertexIndices[nWeights];
  array float weights[nWeights];
  Matrix4x4 matrixOffset;
 }

Frame sky_box {

  FrameTransformMatrix {
    99.0,0.0,0.0,0.0,
    0.0,99.0,0.0,0.0,
    0.0,0.0,-99.0,0.0,
    0.0,0.0,0.0,1.0;;
  }
Mesh {
24;
5.0; 5.0; -5.0;,
-4.99999809265; 5.00000190735; -5.0;,
-5.00000047684; -4.99999904633; -5.0;,
5.0; -5.0; -5.0;,
5.00000238419; 4.99999761581; 5.0;,
4.99999713898; -5.00000286102; 5.0;,
-5.00000190735; -4.99999809265; 5.0;,
-4.99999952316; 5.0; 5.0;,
5.0; 5.0; -5.0;,
5.0; -5.0; -5.0;,
4.99999713898; -5.00000286102; 5.0;,
5.00000238419; 4.99999761581; 5.0;,
5.0; -5.0; -5.0;,
-5.00000047684; -4.99999904633; -5.0;,
-5.00000190735; -4.99999809265; 5.0;,
4.99999713898; -5.00000286102; 5.0;,
-5.00000047684; -4.99999904633; -5.0;,
-4.99999809265; 5.00000190735; -5.0;,
-4.99999952316; 5.0; 5.0;,
-5.00000190735; -4.99999809265; 5.0;,
-4.99999809265; 5.00000190735; -5.0;,
5.0; 5.0; -5.0;,
5.00000238419; 4.99999761581; 5.0;,
-4.99999952316; 5.0; 5.0;;
6;
4; 0, 3, 2, 1;,
4; 4, 7, 6, 5;,
4; 8, 11, 10, 9;,
4; 12, 15, 14, 13;,
4; 16, 19, 18, 17;,
4; 20, 23, 22, 21;;
  MeshMaterialList {
    6;
    6;
    1,
    2,
    3,
    4,
    5,
    4;;
  Material Material {
    0.800000011921; 0.800000011921; 0.800000011921;1.0;;
    0.5;
    1.0; 1.0; 1.0;;
    0.0; 0.0; 0.0;;
  }
  Material Mat5 {
    1.0; 1.0; 1.0; 1.0;;
    1.0;
    1.0; 1.0; 1.0;;
    0.0; 0.0; 0.0;;
  TextureFilename {
    "north.png" ;  }
  }
  Material Mat5 {
    1.0; 1.0; 1.0; 1.0;;
    1.0;
    1.0; 1.0; 1.0;;
    0.0; 0.0; 0.0;;
  TextureFilename {
    "south.png" ;  }
  }
  Material Mat5 {
    1.0; 1.0; 1.0; 1.0;;
    1.0;
    1.0; 1.0; 1.0;;
    0.0; 0.0; 0.0;;
  TextureFilename {
    "east.png" ;  }
  }
  Material Mat5 {
    1.0; 1.0; 1.0; 1.0;;
    1.0;
    1.0; 1.0; 1.0;;
    0.0; 0.0; 0.0;;
  TextureFilename {
    "up.png" ;  }
  }
  Material Mat5 {
    1.0; 1.0; 1.0; 1.0;;
    1.0;
    1.0; 1.0; 1.0;;
    0.0; 0.0; 0.0;;
  TextureFilename {
    "west.png" ;  }
  }
    }
  MeshNormals {
24;
    -0.577349; -0.577349; 0.577349;,
    0.577349; -0.577349; 0.577349;,
    0.577349; 0.577349; 0.577349;,
    -0.577349; 0.577349; 0.577349;,
    -0.577349; -0.577349; -0.577349;,
    -0.577349; 0.577349; -0.577349;,
    0.577349; 0.577349; -0.577349;,
    0.577349; -0.577349; -0.577349;,
    -0.577349; -0.577349; 0.577349;,
    -0.577349; 0.577349; 0.577349;,
    -0.577349; 0.577349; -0.577349;,
    -0.577349; -0.577349; -0.577349;,
    -0.577349; 0.577349; 0.577349;,
    0.577349; 0.577349; 0.577349;,
    0.577349; 0.577349; -0.577349;,
    -0.577349; 0.577349; -0.577349;,
    0.577349; 0.577349; 0.577349;,
    0.577349; -0.577349; 0.577349;,
    0.577349; -0.577349; -0.577349;,
    0.577349; 0.577349; -0.577349;,
    0.577349; -0.577349; 0.577349;,
    -0.577349; -0.577349; 0.577349;,
    -0.577349; -0.577349; -0.577349;,
    0.577349; -0.577349; -0.577349;;
6;
4; 0, 3, 2, 1;,
4; 4, 7, 6, 5;,
4; 8, 11, 10, 9;,
4; 12, 15, 14, 13;,
4; 16, 19, 18, 17;,
4; 20, 23, 22, 21;;
}
MeshTextureCoords {
24;
0.0;-1.0;,
0.0;0.0;,
1.0;0.0;,
1.0;-1.0;,
0.0;-1.0;,
0.0;0.0;,
1.0;0.0;,
1.0;-1.0;,
0.0;-1.0;,
0.0;0.0;,
1.0;0.0;,
1.0;-1.0;,
0.0;-1.0;,
0.0;0.0;,
1.0;0.0;,
1.0;-1.0;,
0.0;-1.0;,
0.0;0.0;,
1.0;0.0;,
1.0;-1.0;,
0.0;-1.0;,
0.0;0.0;,
1.0;0.0;,
1.0;-1.0;;
}
 }
}
