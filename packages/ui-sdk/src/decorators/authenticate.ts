function Logged(target: Function) {
  console.log("here");
}

@Logged
class C {
  m(arg) {}
}

new C().m(1);
// starting m with arguments 1
// ending m
