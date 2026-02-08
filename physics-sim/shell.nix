{ pkgs ? import <nixpkgs> {} }:
(pkgs.buildFHSEnv {
  name = "wpilib-env";
  targetPkgs = pkgs: with pkgs; [
    zlib
    glibc
    libgcc
    stdenv.cc.cc.lib
    jdk17
    gradle
  ];
  runScript = "bash";
}).env
