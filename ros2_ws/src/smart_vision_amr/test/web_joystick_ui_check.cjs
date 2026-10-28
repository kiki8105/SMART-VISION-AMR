const fs = require('fs');
const vm = require('vm');
const assert = require('assert/strict');
class Target {
  constructor(){this.handlers={};this.style={};this.disabled=false;this.dataset={};this.classList={toggle(){}};this.captures=new Set();this.textContent='';}
  addEventListener(name,fn){(this.handlers[name]||=[]).push(fn);}
  dispatch(name,event={}){Object.assign(event,{preventDefault(){}});for(const fn of this.handlers[name]||[])fn(event);}
  replaceChildren(...nodes){this.nodes=nodes;}
  setPointerCapture(id){this.captures.add(id);}
  hasPointerCapture(id){return this.captures.has(id);}
  releasePointerCapture(id){this.captures.delete(id);this.dispatch('lostpointercapture',{pointerId:id});}
  getBoundingClientRect(){return{left:0,top:0,width:170,height:170};}
  focus(){}
}
const elements=Object.fromEntries(['manual-pad','manual-knob','manual-state','manual-reason','manual-linear','manual-angular','manual-limits'].map(id=>['#'+id,new Target()]));
const document=new Target();Object.assign(document,{querySelector:s=>elements[s],createElement:()=>new Target(),createTextNode:text=>({textContent:text}),hidden:false});
const window=new Target(),intervals=new Map(),requests=[];
let intervalId=0;
const sandbox={document,window,console,AbortController,crypto:require('crypto').webcrypto,setTimeout,clearTimeout,setInterval:fn=>{intervals.set(++intervalId,fn);return intervalId;},clearInterval:id=>intervals.delete(id),fetch:(url,options)=>new Promise((resolve,reject)=>requests.push({url,body:JSON.parse(options.body),resolve:state=>resolve({ok:true,json:async()=>({manual:{state}})}),reject})),Math,Date};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync('ros2_ws/src/smart_vision_amr/web/joystick.js','utf8')+'\nglobalThis.joystick=ManualJoystick;',sandbox);
const joystick=sandbox.joystick,pad=elements['#manual-pad'],state=()=>elements['#manual-state'].textContent;
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const ready={manual:{state:'IDLE',available:true,active:false,owner:false},connection:{connected:true},mission:{estop_latched:false}};
const down=(id,x=85,y=15)=>pad.dispatch('pointerdown',{isPrimary:true,button:0,pointerId:id,clientX:x,clientY:y});
(async()=>{
  joystick.setState(ready,{connected:true});assert.equal(pad.disabled,false);
  down(7);assert.equal(requests.length,1);assert.equal(requests[0].body.active,true);assert.equal(requests[0].body.linear,0);assert.equal(requests[0].body.angular,0);
  pad.dispatch('pointerup',{pointerId:7});assert.equal(requests.length,2);assert.equal(requests[1].body.active,false);assert.ok(requests[1].body.seq>requests[0].body.seq);
  requests[0].resolve('ACTIVE');await flush();assert.equal(state(),'임무 재개 대기');assert.equal(intervals.size,0,'late acquire must not restart timer');
  requests[1].resolve('IDLE');await flush();assert.equal(state(),'수동 조종 대기');
  down(8);assert.notEqual(requests[2].body.control_id,requests[0].body.control_id);requests[2].resolve('ACQUIRING');await flush();
  [...intervals.values()][0]();assert.equal(requests[3].body.linear,0,'ACQUIRING must not send motion');requests[3].resolve('ACTIVE');await flush();
  [...intervals.values()][0]();assert.ok(requests[4].body.linear>0);assert.ok(requests[4].body.linear<=0.12);assert.equal(requests[4].body.angular,0);requests[4].resolve('ACTIVE');await flush();
  document.hidden=true;document.dispatch('visibilitychange');assert.equal(requests[5].body.active,false);assert.equal(intervals.size,0);requests[5].resolve('IDLE');await flush();document.hidden=false;
  pad.dispatch('keydown',{key:'ArrowLeft'});requests[6].resolve('ACTIVE');await flush();[...intervals.values()][0]();assert.equal(requests[7].body.angular,.6);assert.equal(requests[7].body.linear,0);requests[7].resolve('ACTIVE');await flush();
  window.dispatch('keyup',{key:'ArrowLeft'});assert.equal(requests[8].body.active,false);requests[8].resolve('IDLE');await flush();
  down(9);requests[9].resolve('ACTIVE');await flush();joystick.setState({...ready,mission:{estop_latched:true}},{connected:true});assert.equal(requests[10].body.active,false);requests[10].resolve('IDLE');await flush();assert.equal(pad.disabled,true);
  joystick.setState(ready,{connected:true});down(10);requests[11].resolve('ACTIVE');await flush();window.dispatch('blur');assert.equal(requests[12].body.active,false);requests[12].resolve('IDLE');await flush();
  down(11);requests[13].resolve('IDLE');await flush();assert.equal(requests[14].body.active,false);assert.equal(intervals.size,0,'expired gesture must close instead of retrying forever');requests[14].resolve('IDLE');await flush();
  console.log('PASS: zero-before-ack, release-before-acquire, fresh control IDs, sequence order, 10Hz motion, direction/clamps, hidden/blur/estop release, keyboard release.');
})().catch(error=>{console.error(error);process.exitCode=1;});
