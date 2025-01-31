import { Component } from '@angular/core';

@Component({
  selector: 'app-rosblocks-help',
  templateUrl: './rosblocks-help.component.html',
  styleUrls: ['./rosblocks-help.component.css']
})
export class RosblocksHelpComponent {
  dropdowns: { [key: number]: boolean } = {};
  selectedVideo: string | null = null;

  onSearch(event: any): void {
    console.log('Buscando:', event.target.value);
  }

  toggleDropdown(id: number): void {
    this.dropdowns[id] = !this.dropdowns[id];
  }

  selectVideo(videoName: string): void {
    switch(videoName) {
      case 'video1':
        this.selectedVideo = 'assets/videos/video1.mp4';
        break;
      case 'video2':
        this.selectedVideo = 'assets/videos/video2.mp4';
        break;
      case 'video3':
        this.selectedVideo = 'assets/videos/video3.mp4';
        break;
      case 'videoA':
        this.selectedVideo = 'assets/videos/videoA.mp4';
        break;
      case 'videoB':
        this.selectedVideo = 'assets/videos/videoB.mp4';
        break;
      case 'videoC':
        this.selectedVideo = 'assets/videos/videoC.mp4';
        break;
      case 'videoX':
        this.selectedVideo = 'assets/videos/videoX.mp4';
        break;
      case 'videoY':
        this.selectedVideo = 'assets/videos/videoY.mp4';
        break;
      case 'videoZ':
        this.selectedVideo = 'assets/videos/videoZ.mp4';
        break;
      default:
        this.selectedVideo = null;
    }
  }
}
