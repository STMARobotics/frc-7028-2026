{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = [
    pkgs.eigen
    pkgs.clang
    pkgs.ninja
    pkgs.lldb
    pkgs.cmake
  ];
}
