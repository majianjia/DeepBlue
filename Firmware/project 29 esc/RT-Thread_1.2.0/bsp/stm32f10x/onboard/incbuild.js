function incbuild() {
  var file = "buildnum.h";
  var text = "#define BUILDNUMBER ";
  var s = CWSys.readStringFromFile(file);
  var n;
  if (s == undefined)
    n = 1;
  else
    n = eval(s.substring(text.length)) + 1;
  CWSys.writeStringToFile(file, text + n);
}
