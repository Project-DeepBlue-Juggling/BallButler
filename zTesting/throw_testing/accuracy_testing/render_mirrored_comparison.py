"""Render the saved comparison as a portable PNG and UTF-8 SVG; requires Pillow."""
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent


def render():
    results = json.loads((ROOT / 'mirrored_hand_results.json').read_text())
    image = Image.new('RGB', (1600, 940), 'white')
    draw = ImageDraw.Draw(image)
    def font(size):
        for name in ('DejaVuSans.ttf', 'C:/Windows/Fonts/arial.ttf'):
            try:
                return ImageFont.truetype(name, size)
            except OSError:
                pass
        return ImageFont.load_default()
    small, normal, title = font(17), font(22), font(30)
    draw.text((40, 20), 'Mirrored hand: physics prediction vs archived landings', fill='#172b40', font=title)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="940">',
           '<rect width="1600" height="940" fill="white"/>']
    def text(x,y,t,size=17):
        svg.append(f'<text x="{x}" y="{y+size}" font-family="Arial" font-size="{size}" fill="#172b40">{t}</text>')
    text(40,20,'Mirrored hand: physics prediction vs archived landings',30)
    for k,(key,label) in enumerate((('uncorrected','No affine: 41 throws, 21 targets'),('affine_validation','Deployed affine: 32 throws, 16 targets'))):
        x0,y0=80+k*790,135
        scale=.41
        def point(x,y): return x0+x*scale,y0+(1100-y)*scale
        draw.text((x0,85),label,fill='#172b40',font=normal)
        text(x0,85,label,22)
        for xx in range(0,1601,200):
            x,y=point(xx,-200)
            draw.line((x,y0,x,y),fill='#e1e7ed')
            draw.text((x-15,y+12),str(xx),fill='#516475',font=small)
        for yy in range(-200,1101,200):
            x,y=point(0,yy)
            draw.line((x,y,x0+1600*scale,y),fill='#e1e7ed')
            draw.text((x-55,y-9),str(yy),fill='#516475',font=small)
        for cell in results[key]['cell_results']:
            tx,ty=point(*cell['target']); mx,my=point(*cell['measured']); px,py=point(*cell['simulated'])
            draw.line((tx,ty,mx,my),fill='#aeb9c4',width=2)
            draw.line((tx-6,ty,tx+6,ty),fill='#172b40',width=2)
            draw.line((tx,ty-6,tx,ty+6),fill='#172b40',width=2)
            draw.ellipse((mx-5,my-5,mx+5,my+5),fill='#d67915')
            draw.ellipse((px-8,py-8,px+8,py+8),outline='#087ea4',width=3)
            svg.extend([f'<path d="M{tx},{ty} L{mx},{my}" stroke="#aaa"/>',
                        f'<path d="M{tx-6},{ty} h12 M{tx},{ty-6} v12" stroke="#172b40" stroke-width="2"/>',
                        f'<circle cx="{mx}" cy="{my}" r="5" fill="#d67915"/>',
                        f'<circle cx="{px}" cy="{py}" r="8" fill="none" stroke="#087ea4" stroke-width="3"/>'])
        metric=results[key]['measured_minus_simulated']['mean_mm']
        draw.text((x0,735),f'Mean measured-to-predicted distance: {metric:.1f} mm',fill='#172b40',font=normal)
        text(x0,735,f'Mean measured-to-predicted distance: {metric:.1f} mm',22)
        draw.text((x0+170,710),'BB-local X (mm); vertical axis: Y (mm)',fill='#516475',font=small)
    legend='+ Target     Orange dot: measured mean     Blue ring: mirrored-hand prediction'
    draw.text((80,810),legend,fill='#172b40',font=normal); text(80,810,legend,22)
    note='Equal X/Y scale. Existing commands, physical s = +105.65 mm. No fitted physical parameters.'
    draw.text((80,850),note,fill='#516475',font=small); text(80,850,note)
    svg.append('</svg>')
    (ROOT/'mirrored_hand_comparison.svg').write_text('\n'.join(svg),encoding='utf-8')
    image.save(ROOT/'mirrored_hand_comparison.png')


if __name__ == '__main__':
    render()
