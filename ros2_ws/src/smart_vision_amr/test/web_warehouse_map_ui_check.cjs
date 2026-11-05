// The actual SVG renderer must rotate every warehouse layer together, with no ROS changes.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const drawing=source.slice(source.indexOf('  function polygonPath('),source.indexOf('  function updateCamera(){'));
const elements=new Map();
function element(name='div',attrs={}){
  const classes=new Set();
  return{name,attrs,children:[],hidden:false,textContent:'',dataset:{},clientWidth:960,clientHeight:480,
    classList:{toggle(key,on){on?classes.add(key):classes.delete(key);},contains:key=>classes.has(key)},
    setAttribute(key,value){this.attrs[key]=String(value);},replaceChildren(){this.children=[];},
    append(...items){this.children.push(...items);}};
}
const $=selector=>{if(!elements.has(selector))elements.set(selector,element());return elements.get(selector);};
const svg=(name,attrs={},text='')=>{const node=element(name,attrs);node.textContent=text;return node;};
const markers=Array.from({length:11},(_,id)=>({id,name:id?`Sector ${id}`:'HOME',
  x:id?1.7:2.8,y:id?-id*2.5:1,yaw:id?0:-Math.PI/2,camera:'rear'}));
const snapshot={connection:{domain_id:178,use_sim_time:true},
  map:{width:200,height:610,resolution:.05,origin:{x:-5,y:-27.5,yaw:0},version:7,
    configured_match:true,raw_available:true},markers,
  mission:{state:'NAVIGATING',current_marker_id:10},navigation:{planner_status:'NAVIGATING'},
  path:[{...markers[0]},{...markers[10]}],robot:{pose:{...markers[0]}},
  observed_obstacles:{map_version:7,cells:[1200],age:.1},
  remembered_obstacles:{map_version:7,cells:[1400]}};
const layout={width:200,height:610,version:7,style:'architectural',display_rotation_deg:0,
  free_paths:[[[0,0],[200,0],[200,610],[0,610]]],occupied_paths:[],obstacle_paths:[]};
const sandbox={$,svg,snapshot,mapLayout:layout,mapVersion:7,mapLayoutLoading:false,
  showRawMap:false,mapImageUrl:'/api/map.png',lastPath:[],lastMonitoredPathGoal:null,draft:[],
  currentMission:()=>snapshot.mission,homepoint:()=>markers[0],
  markerLabel:id=>markers.find(marker=>marker.id===id).name,terminal:new Set()};
vm.createContext(sandbox);vm.runInContext(drawing+'\nglobalThis.draw=drawMap;',sandbox);
function descendants(root,name){return root.children.flatMap(child=>[
  ...(child.name===name?[child]:[]),...descendants(child,name)]);}
const canvas=$('#map-svg'),before=JSON.stringify(snapshot);
for(const raw of [false,true]){
  sandbox.showRawMap=raw;sandbox.draw();
  assert.equal($('#map-area').classList.contains('warehouse-map-mode'),true);
  const artwork=canvas.children[0],memory=canvas.children[1],live=canvas.children[2];
  assert.match(artwork.attrs.transform,/rotate\(-90 100 305\)$/);
  assert.equal(memory.attrs.transform,artwork.attrs.transform,'memory and map share one transform');
  assert.equal(live.attrs.transform,artwork.attrs.transform,'live pixels and map share one transform');
  const cards=descendants(canvas,'rect').filter(node=>node.attrs.class==='map-location-card');
  assert.equal(cards.length,11);assert.ok(cards.every(card=>card.attrs.height===24&&card.attrs.width<=108));
  assert.ok(cards.slice(1).every(card=>card.attrs.width===48),'ten sector labels are compact');
  assert.deepEqual(descendants(canvas,'text').filter(node=>node.attrs.class==='map-location-name')
    .map(node=>node.textContent),['HOME',...Array.from({length:10},(_,id)=>`S${id+1}`)]);
  assert.ok(descendants(canvas,'title').some(node=>node.textContent.includes('Sector 10 · ArUco ID 10')),
    'full names remain accessible in tooltips');
  const locations=descendants(canvas,'g').filter(node=>String(node.attrs.class||'').startsWith('map-location'));
  const pins=locations.map(group=>group.children.find(node=>node.attrs.class==='map-location-point'));
  assert.ok(pins[0].attrs.cx<pins[10].attrs.cx,'HOME is on the left, sectors extend to the right');
  const route=descendants(canvas,'polyline').find(node=>node.attrs['aria-label']==='계획 경로');
  const endpoints=route.attrs.points.split(' ').map(pair=>pair.split(',').map(Number));
  for(const [index,pin] of [[0,pins[0]],[1,pins[10]]]){
    assert.ok(Math.abs(endpoints[index][0]-pin.attrs.cx)<1e-8&&Math.abs(endpoints[index][1]-pin.attrs.cy)<1e-8,
      'route endpoints stay aligned with sector positions');
  }
  const robot=descendants(canvas,'g').find(node=>node.attrs['aria-label']==='현재 차량 위치');
  assert.equal(robot.attrs.transform,`translate(${pins[0].attrs.cx},${pins[0].attrs.cy}) rotate(0)`,
    'robot position and heading share the display rotation');
  const heading=locations[10].children.find(node=>node.attrs.class==='map-location-heading');
  assert.match(heading.attrs.transform,/rotate\(-90\)$/,'actual sector yaw is rotated only for display');
}
assert.equal(JSON.stringify(snapshot),before,'rendering never changes world, mission, paths or target yaw');
sandbox.mapLayout=null;sandbox.draw();
assert.match(canvas.children[0].attrs.transform,/rotate\(-90 100 305\)$/,
  'raw fallback remains landscape while the floorplan loads');
sandbox.mapLayout=layout;layout.display_rotation_deg=-1.5;sandbox.showRawMap=false;
for(const [domain,sim] of [[16,false],[176,true],[178,false]]){
  snapshot.connection={domain_id:domain,use_sim_time:sim};sandbox.draw();
  assert.equal($('#map-area').classList.contains('warehouse-map-mode'),false);
  assert.match(canvas.children[0].attrs.transform,/rotate\(-1\.5 100 305\)$/,
    'classroom and physical maps retain their configured display rotation');
  assert.ok(descendants(canvas,'rect').filter(node=>node.attrs.class==='map-location-card')
    .every(card=>card.attrs.height===32),'existing map label sizing is unchanged');
}
console.log('Warehouse landscape map, compact labels and all-layer registration PASS');
