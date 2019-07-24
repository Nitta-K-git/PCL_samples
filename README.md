# PCL_samples
PCL basic samples

# 環境構築

### 参考リンク

- http://unanancyowen.com/pcl191
- http://unanancyowen.com/pcl-with-qt/

## 手順

1. CMakeが無い場合はインストール(https://cmake.org/download/)
2. PCLの[PCL 1.9.1 All-in-one Installer MSVC2017 x64](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.9.1/PCL-1.9.1-AllInOne-msvc2017-win64.exe)をダウンロードし、必要なライブラリをインストールする
   - ライブラリをインストールしない場合は自分でビルドする必要がある
3. Qt上でVTKをウィジェットとして使いたい場合
   - 以下のページを参考にビルドを行う。ソースはなるべく最新の方がよい
     - [Building QVTK(VTK7+Qt5) with Visual Studio](https://gist.github.com/UnaNancyOwen/77d61f9f21376c9b59fc#building-qvtkvtk7qt5-with-visual-studio)
     - [Building VTK 8.2.0 with Visual Studio](https://gist.github.com/UnaNancyOwen/9d16060714ba9b28f90e#building-vtk-820-with-visual-studio)
     - [Building PCL 1.9.1 with Visual Studio](https://gist.github.com/UnaNancyOwen/59319050d53c137ca8f3#building-pcl-191-with-visual-studio)
   - PCLをビルドするときはVTK以外はPCLでインストールした3rdPatyのディレクトリを指定してもよい

### Visual Studio 2017 64bit版でビルドしたときの注意点

- ソースファイルをデフォルトではshiftJISと見なすため、UTF-8のファイルではエラーが出る。CMAKEの変数CMAKE_CXX_FLAGSに/utf-8を追加する(cmake-guiでやると楽)。
- cmakeのconfigure中にパスが設定されてないと途中でエラーが出て止まる。エラーが出たら該当の箇所＋埋められる変数を埋めて再度configureを実行する
  - 最初のconfigureでは処理が最後まで行かないので出てこない変数があるが、設定埋めてるうちに出てくる
- コンパイル中によく分からないエラーが出てくる場合は大抵cmakeの設定がおかしいことが多い
- ALL_BUILDしたら、INSTALLする前に想定通りのファイルが生成されているか確認する(libのデバッグモードでサフィックスが付いてなかったりするとリリースモードと混ざってしまう)

## 動作確認

Qtでの動作確認をする場合は`Qt_visualizer_demo`のディレクトリ内で以下のコマンド実行する。

```
md build
cd build
cmake .. -G"Visual Studio 15 2017 Win64"
cmake --build . --config Release
```

`cmake .. -G"Visual Studio 15 2017 Win64"`の箇所は使用するコンパイラに合わせて変える。

使えるコンパイラはcmake --helpで確認可能。

---

## Qt_visualizer_demo

![](img/Qt_visualizer_demo.png)

PCL+Qt+VTKのサンプル

[PCLの公式ページ](http://pointclouds.org/documentation/tutorials/qt_visualizer.php)とhttp://unanancyowen.com/pcl-with-qt/にサンプルがあるが、どちらも最新の環境ではmetaObjectのリンクエラーなどで動かないため注意。

[Qtのcmakeサンプル](https://doc.qt.io/qt-5/cmake-manual.html)のページを参考にCMakeLists.txtを作成。

---

### memo

VTKマウスピッキング・テキスト描画 http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

メッシュデータ pcl::PolygonMesh

点群データ pcl::PointCloud<PointT>

C:\Program Files\PCL\include\pcl-1.9\pcl\visualization\pcl_visualizer.h

